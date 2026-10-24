"""
route_finder.py — O-D 航路规划 (Origin-Destination Route Finder)

用途:
    find_route(起点ICAO, 终点ICAO) -> list[str]
    列表结构: [起飞机场, SID终点, ...中转航路点..., STAR起点, 降落机场]
        index 1  = SID 终点(离场程序交给航路的点)
        index -2 = STAR 起点(进近程序从航路接收的点)
    例: ["ZBAA", "MUGLO", "PAMRU", "PANKI", ..., "DADAT", "AND", "ZSPD"]
    SID/STAR 的中间程序点不展开, 只保留与航路衔接的端点.

数据来源 (X-Plane 12 导航数据库, 首次加载约 0.3 秒, 之后走缓存):
    1. earth_awy.dat     航路网络: "起点 区域 类型 终点 区域 类型 单双向 基础FL 最高FL 航路名"
    2. earth_fix.dat     航路点坐标: "纬度 经度 名称 所属机场 区域 类型ID 名称"
    3. earth_aptmeta.dat 机场坐标:   "ICAO 区域 纬度 经度 ..."
    4. CIFP/<ICAO>.dat   SID/STAR 程序 (SID 与 STAR 的列定义不同, 见 _procedure_points)
    5. earth_nav.dat     导航台(VOR/NDB/ILS)坐标, CIFP 端点查不到时兜底

算法流程:
    1. 读起降机场坐标
    2. 解析两端 CIFP, 按程序名分组, 再按"序号重置"切成独立航段块
       (同一程序名常含多条跑道/过渡段, 合并会得到来回跳动的点列)
    3. 每块单独提取点列, 按坐标距离、方位角、单调性、完整性筛选成候选
    4. 端点不在航路图上时, 反向可达性搜索(_reach_map)一次性找出"能连通目标"的
       接入点, 避免 6x6 组合各跑一次全图 A*
    5. 对 (SID终点, STAR起点) 组合做 A* (允许逆行但加罚), 再按
       总航程 + 逆行惩罚 + 接入代价 + 绕飞惩罚 评分, 绕飞比 > MAX_DETOUR_RATIO 直接淘汰
    6. 拼成 [机场, SID终点, 中转航路点..., STAR起点, 机场]

关键防御(都是实测踩出来的):
    - 同名航路点: 必须带"参考坐标"解析, 否则 DANDE 会取到加勒比海同名点,
      A* 启发式失效, 单次查询从 50ms 膨胀到 4 秒
    - 三字母导航台: LBN/LXI/SAS 等在欧洲与亚洲重名, 不能按名字取第一条
    - 目的地坐标: A* 的 target_ref 必须是真实机场坐标, 否则同样导致启发式失效
"""

import heapq
import math
import os
import re

# ============================ 路径常量 ============================

# X-Plane 12 导航数据库根目录。
# 换机器/换安装位置时：设环境变量 XP_DATA_DIR，或直接改下面这一行。
XP_DATA_DIR = os.environ.get(
    "XP_DATA_DIR",
    os.path.expanduser("~/X-Plane 12/Resources/default data"),
)
CIFP_DIR = os.path.join(XP_DATA_DIR, "CIFP")          # 每个机场一个 .dat 文件
AWY_PATH = os.path.join(XP_DATA_DIR, "earth_awy.dat")  # 航路网络
FIX_PATH = os.path.join(XP_DATA_DIR, "earth_fix.dat")  # 航路点坐标
APT_PATH = os.path.join(XP_DATA_DIR, "earth_aptmeta.dat")  # 机场坐标
NAV_PATH = os.path.join(XP_DATA_DIR, "earth_nav.dat")  # 导航台(坐标兜底)

# ============================ 可调参数 ============================

ENDPOINT_SEARCH_NM = 150.0   # 程序端点不在航路图上时, 就近接入航路网的最大半径
BEARING_WINDOW_DEG = 60.0    # 程序端点必须在机场的"背离方向"±此角度内
BEARING_MIN_DIST_NM = 40.0   # 端点离机场超过此距离才用方位判据(近距离方位误差过大)
MAX_FIX_JUMP_NM = 250.0      # 相邻程序点距离上限, 超过说明匹配到了别处的同名航路点
PROCEDURE_RADIUS_NM = 220.0  # SID/STAR 所有点必须在机场此半径内, 否则整段程序作废
PROCEDURE_END_NM = 45.0      # STAR 末点离机场的距离上限; 欧洲常见 25-30nm 的过渡点(LOWW 的 BALAD/NERDU),
                             # 中国多为 10nm 内, 用 45nm 兜住两种风格, 真正的完整性由"末点收敛"判断
REVERSE_PENALTY = 0.25       # 逆行航路的代价放大比例(0.25 = 同样的边多花 25% 距离)
ACCESS_CANDIDATES = 6        # 端点接入航路网时尝试的候选点数(应对导航数据的分量割裂)
MAX_REACH_NM = 12000.0       # 反向可达性搜索的最大扩展距离(洲际航线 5000-7000nm 需覆盖)
ACCESS_DIST_LIMIT_NM = 260.0 # 两端接入距离之和的上限, 超过说明程序端点离航路太远
MAX_DETOUR_RATIO = 1.35      # 绕飞比上限(总航程/大圆距离), 超过视为不可接受的绕圈航路
DETOUR_SOFT_RATIO = 1.12     # 评分时开始加罚的绕飞比, 真实航路一般在 1.02-1.10

# ============================ 懒加载的数据集 ============================

_CACHE = {}  # 全局缓存字典, 放解析好的图, 避免重复读大文件


def _cached(key, builder):
    """惰性求值 + 缓存: 第一次调用 builder(), 之后直接返回缓存值."""
    if key not in _CACHE:
        _CACHE[key] = builder()
    return _CACHE[key]


# ============================ 通用坐标工具 ============================


def _nm_distance(lat1, lon1, lat2, lon2):
    """两点大圆距离(海里), 使用等距圆柱近似, 航线尺度上误差 < 0.5%."""
    mean_lat = math.radians((lat1 + lat2) / 2.0)      # 取中纬度做经度收缩
    dlat = (lat1 - lat2) * 60.0                        # 1 度纬度 ≈ 60 海里
    dlon = (lon1 - lon2) * 60.0 * math.cos(mean_lat)   # 经度按 cos(纬度) 收缩
    return math.hypot(dlat, dlon)


def _along_track(lat, lon, lat1, lon1, lat2, lon2):
    """
    点 (lat, lon) 沿大圆航线 lat1,lon1 -> lat2,lon2 的投影里程(海里, 从起点算起).
    用于判断航路点是否沿路线"向前推进", 防止先飞到终点之后又倒回来.
    """
    # 用局部平面近似: 航线方向单位向量与点到起点向量做点积, 结果即为纵向投影
    ax, ay = lon1, lat1
    bx, by = lon2, lat2
    dx, dy = bx - ax, (by - ay) * math.cos(math.radians((lat1 + lat2) / 2.0))
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return 0.0
    px, py = (lon - ax) * math.cos(math.radians((lat1 + lat2) / 2.0)), lat - ay
    return (px * dx + py * dy) / L * 60.0          # 60 海里/度


def _bearing(lat1, lon1, lat2, lon2):
    """起点到终点的初始方位角(度, 0=正北, 顺时针)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _angle_diff(a, b):
    """
    两个方位角之差, 归一化到 [0, 180] 度.

    注意取模顺序: 必须先 (a-b) % 360 再取绝对值, 或先取绝对值再 % 360 都可,
    但不能写成 abs((a-b) % 360) 之后再 360-d —— 那样 135.8 与 323.2 会算出
    172.6 度(实际只差 172.6 是错的, 正确差值 172.6 恰好是 360-187.4 的互补,
    真正的短角是 187.4 -> 172.6, 但方向语义颠倒了). 这里直接用最短夹角公式.
    """
    d = (a - b) % 360.0                   # 归一到 [0, 360)
    return d if d <= 180.0 else 360.0 - d # 取短角, 结果在 [0, 180]


# ============================ 数据加载 ============================


def _load_airports():
    """解析 earth_aptmeta.dat -> {ICAO: (lat, lon)}. 行格式: ICAO 区域 纬度 经度 ..."""
    table = {}
    with open(APT_PATH, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 4 or not re.fullmatch(r"[A-Z0-9]{4}", parts[0]):
                continue
            try:
                table[parts[0]] = (float(parts[2]), float(parts[3]))
            except ValueError:
                continue
    return table


def _load_navaid_coords():
    """
    解析 earth_nav.dat 取导航台坐标 -> {名称: [(lat, lon, 区域), ...]}

    导航台名多为 3 个字母, 全球重名极多(SFD 在英国和美国都有, WOD 在欧美多处),
    因此值必须是列表, 由调用方按参考点距离挑选, 不能只留第一条.

    行格式: 类型码 纬度 经度 海拔 频率 覆盖半径 磁差 名称 关联机场 区域 ...
    类型码 2/3=VOR, 4/6=NDB, 12/13/14/15/16=各类 DME/ILS/航路点
    """
    table = {}
    nav_codes = ("2", "3", "4", "5", "6", "12", "13", "14", "15", "16")
    with open(NAV_PATH, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 8 or parts[0] not in nav_codes:
                continue
            try:
                lat, lon = float(parts[1]), float(parts[2])
            except ValueError:
                continue
            region = parts[9] if len(parts) > 9 else ""
            table.setdefault(parts[7], []).append((lat, lon, region))
    return table


def _load_fixes():
    """
    解析 earth_fix.dat -> {航路点: [(lat, lon, 所属机场, 区域), ...]}

    同名航路点在全球可能有多份(AA412 同时存在于 ZBAA / DNAA / MMAA / NZAA),
    所以值必须是列表, 由调用方按机场/区域/距离挑选正确的那一份.

    行格式: 纬度 经度 名称 所属机场(或 ENRT) 区域码 类型ID 名称
    """
    table = {}
    with open(FIX_PATH, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 5 or parts[0] == "I" or parts[0] == "1200":
                continue
            try:
                lat, lon = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            # 每项: 纬度, 经度, 归属机场(ENRT 表示航路点), 区域码
            table.setdefault(parts[2], []).append((lat, lon, parts[3], parts[4]))
    return table


def _nav_coord(name, near=None):
    """
    从同名导航台中挑出正确坐标.
    导航台名只有 3 个字母, 全球重名严重(如 SFD 在美国/英国, WOD 在欧美多处),
    只取第一条会把航路连到另一个大洲. 规则: 有参考点时取最近且不超过 MAX_FIX_JUMP_NM 的.
    """
    entries = _get_navaids().get(name)
    if not entries:
        return None
    if near:
        # 二步筛选: 优先附近(方位合理)的台, 都没有就近的再取全局最近的一个.
        ranked = sorted(entries, key=lambda e: _nm_distance(near[0], near[1], e[0], e[1]))
        return (ranked[0][0], ranked[0][1])
    return entries[0]


def _node_coord(name, near=None):
    """
    航路图节点坐标访问器: 返回 (lat, lon) 或 None.
    near 为参考坐标(通常是上一个已确认的点), 用来消除同名歧义.
    """
    # 四个大写字母是机场 ICAO, 先查机场表, 否则机场会被同名航路点/导航台抢走
    if len(name) == 4 and name.isupper():
        apt = _get_airports().get(name)
        if apt is not None:
            return apt
    coord = _fix_coord(name, near=near)     # 再查航路点表 (已返回二元组)
    if coord is not None:
        return coord
    nav = _nav_coord(name, near)            # 最后查导航台表
    if nav is not None:
        return (nav[0], nav[1])             # 去掉区域码, 统一成二元组
    return None


def _fix_coord(name, airport=None, near=None):
    """
    从同名航路点中挑出正确坐标.
    优先级: 归属机场 == airport  >  离 near 最近且不超过 MAX_FIX_JUMP_NM  >  第一条.
    airport 为机场 ICAO, near 为 (lat, lon) 参考点.
    """
    entries = _get_fixes().get(name)
    if not entries:
        return None
    if airport:
        for lat, lon, apt, _ in entries:
            if apt == airport:
                return (lat, lon)                     # 程序所属机场的同名点最可信
    if near:
        # 同名点离参考点过远说明是别处的同名航路点, 视为解析失败.
        # 但"解析失败"不等于"没有坐标": 有些三字母台名只在中国区存在, 而被欧洲航段
        # 引用(说明航路数据本身把两个不同实体用了同一个名字). 此时退而取全局最近的
        # 一个, 让搜索继续, 由最终的绕飞比与纵向单调性把关.
        ranked = sorted(entries, key=lambda e: _nm_distance(near[0], near[1], e[0], e[1]))
        best = ranked[0]
        if _nm_distance(near[0], near[1], best[0], best[1]) <= MAX_FIX_JUMP_NM:
            return (best[0], best[1])
        if len(ranked) == 1:
            return (best[0], best[1])          # 全局只有一个, 没有歧义可言
        return (best[0], best[1])              # 都不在附近时仍取最近的一个
    lat, lon, _, _ = entries[0]
    return (lat, lon)


def _load_airways():
    """
    解析 earth_awy.dat -> 双向邻接表 {航路点: {邻居: [(航路名, 最低FL, 最高FL), ...]}}
    行格式: 起点 区域 类型 终点 区域 类型 单双向(1单向/2双向) 基础FL 最高FL 航路名
    同一对点可能被多条航路连接, 所以邻居值用列表保存全部航路名.
    """
    graph = {}
    with open(AWY_PATH, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.split()
            # 跳过文件头("I" 和 "1100 Version..." 等)以及字段不足的坏行
            if len(parts) < 11 or parts[0] == "I" or not parts[2].isdigit():
                continue
            a, b = parts[0], parts[3]                    # 航段两端
            two_way = parts[6] == "2"                    # 2 = 双向, 1 = 单向
            low_fl, high_fl = parts[8], parts[9]         # 巡航高度上下限
            airway = parts[10]                           # 航路名
            graph.setdefault(a, {}).setdefault(b, []).append((airway, low_fl, high_fl))
            if two_way:                                  # 双向航路补反向边
                graph.setdefault(b, {}).setdefault(a, []).append((airway, low_fl, high_fl))
    return graph


def _get_airports():
    """返回缓存的机场坐标表 {ICAO: (lat, lon)}."""
    return _cached("airports", _load_airports)


def _get_navaids():
    """返回缓存的导航台坐标表 {名称: (lat, lon)}."""
    return _cached("navaids", _load_navaid_coords)


def _get_fixes():
    """返回缓存的航路点表 {名称: [(lat, lon, 机场, 区域), ...]}."""
    return _cached("fixes", _load_fixes)


def _get_airways():
    return _cached("airways", _load_airways)


# ============================ CIFP 程序解析 ============================

# CIFP 里 SID/STAR 行的固定列号 (逗号分隔后的下标)
_COL_TYPE = 0      # "SID:010" / "STAR:010"
_COL_NAME = 2      # 程序名, 如 BOT8XD / AVB8ZA
_COL_RUNWAY = 3    # 关联跑道 (SID/STAR 均为第 3 列), "RW18L" 或空(全跑道通用)
_COL_FIX = 4       # 航段终点航路点
_COL_PATH_TERM = 11  # 航段类型: IF/TF/DF/CA/CF/VA/VM ...
# 以下两类列在 SID 与 STAR 中不同, 必须分开读 (X-Plane CIFP 规范)
_SID_EXIT_COL = 8    # SID 的第 8 列是 "退出点"(出口航路点), 过渡段标志
_STAR_ENTRY_COL = 8  # STAR 的第 8 列是 "限制类型"(如 EECH), 不是入口点


def _read_cifp_rows(icao):
    """读取某机场 CIFP 文件, 返回 (SID行列表, STAR行列表), 每个元素是 split 后的字段列表."""
    path = os.path.join(CIFP_DIR, f"{icao.upper()}.dat")
    sids, stars = [], []
    if not os.path.exists(path):
        return sids, stars
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("SID:"):
                sids.append(line.rstrip("\n").rstrip(";").split(","))
            elif line.startswith("STAR:"):
                stars.append(line.rstrip("\n").rstrip(";").split(","))
    return sids, stars


def _group_procedures(rows):
    """
    把扁平的程序行按 "程序名" 分组, 并在组内按"独立航段块"再切分.

    CIFP 一个程序名下往往包含多个互不相连的块, 每块序号都从 010 重新开始:
        STAR:010 DEBH1A     , DEBHI  IF   <- 块1: 从 DEBHI 一路到跑道
        ...
        STAR:150 DEBH1A     , DF416  TF
        STAR:010 DEBH1A RW25C, DF416 IF  <- 块2: 换跑道, 序号重置
        STAR:030 DEBH1A RW25C, DF526 CF
    若把同一个名字的所有行拼成一条, 航路会变成 "83nm -> 35nm -> 66nm -> 35nm" 的
    来回跳动, 单调性检查必然失败, 整组 STAR 就被误判为"残缺程序"淘汰.

    返回 {程序名: {"blocks": [ {"runways": set(), "rows": [...]}, ... ]}}
    每个 block 是一条首尾相连的独立航段.
    """
    grouped = {}
    for cols in rows:
        if len(cols) <= _COL_PATH_TERM:
            continue
        name = cols[_COL_NAME].strip()                 # 程序名, 去掉尾部空格
        if not name:
            continue
        runway = cols[_COL_RUNWAY].strip()             # "RW18L" 或空串(全跑道通用)
        runway = runway.replace("RW", "")
        seq = int(re.sub(r"\D", "", cols[0]) or 0)      # 去掉 "SID:"/"STAR:" 前缀取序号
        blocks = grouped.setdefault(name, [])
        # 序号不再递增(或小于等于上一行) => 新块开始; 跑道变化也视为新块
        if not blocks or seq <= blocks[-1]["last_seq"] or blocks[-1]["runway"] != runway:
            blocks.append({"runways": set(), "rows": [], "last_seq": 0, "runway": runway})
        block = blocks[-1]
        block["last_seq"] = seq
        if runway:
            block["runways"].add(runway)
        block["rows"].append(cols)

    # 块内去重(同一行可能因过渡段重复出现), 保持原顺序
    for blocks in grouped.values():
        for block in blocks:
            seen = set()
            unique = []
            for cols in block["rows"]:
                key = (cols[0], tuple(c.strip() for c in cols[4:12]))
                if key in seen:
                    continue
                seen.add(key)
                unique.append(cols)
            block["rows"] = unique
    return grouped


def _procedure_points(cols_list, name, navaids, airport=None, radius_nm=None):
    """
    把程序行列表转成 [(航路点, (lat, lon)), ...], 坐标查 fix 后查 nav, 查不到跳过.

    两处必须特殊处理, 否则会拿到错误坐标或错误端点:
      1. 跑道延长点(如 DE18L / RW18B) 不是真实航路点, earth_fix/earth_nav 里的
         同名字段可能是别处的导航台(DE18L 曾解析到 127E 的远处), 一律跳过.
      2. CIFP 的行内 "next fix" 有时是跑道虚拟点, 官方接线点在第 8 列:
         此时用第 8 列的真实航路点接续程序.
    """
    points = []
    # 起点参考: 第一个点必须靠近本机场, 否则同名 VOR/NDB 会被匹配到别的大洲
    origin_ref = _get_airports().get(airport) if airport else None
    prev_coord = origin_ref
    for cols in cols_list:
        fix = cols[_COL_FIX].strip()

        # 跑道虚拟点(DE18L / RW18L 等)没有可用坐标, 改用第 8 列的真实航路点
        if not fix or fix.startswith("RW") or re.fullmatch(r"(DE|DER|FF|RW)?\d{2,3}[LRC]?", fix):
            alt_fix = cols[8].strip()
            if alt_fix and not alt_fix.startswith("RW"):
                fix = alt_fix
            else:
                continue

        # 先查航路点表(可按机场消歧), 再查导航台表(必须带参考点, 否则同名台会跨洲)
        coord = _fix_coord(fix, airport, prev_coord) or _nav_coord(fix, prev_coord or origin_ref)
        if coord is None:
            alt_fix = cols[8].strip()                # 第 4 列无坐标时用第 8 列兜底
            if alt_fix and alt_fix != fix and not alt_fix.startswith("RW"):
                coord = _fix_coord(alt_fix, airport, prev_coord) or _nav_coord(alt_fix, prev_coord or origin_ref)
                if coord is not None:
                    fix = alt_fix
            if coord is None:
                continue
        prev_coord = coord
        # 连续重复的航路点只保留一个 (CIFP 里有 TF 接 TF 的重复情形)
        if points and points[-1][0] == fix:
            continue
        # 相邻点跳变或离机场过远 -> 匹配到了别处同名点, 到此为止(保留前面可信的点)
        if prev_coord and _nm_distance(prev_coord[0], prev_coord[1], coord[0], coord[1]) > MAX_FIX_JUMP_NM:
            break
        if radius_nm and _nm_distance(origin_ref[0], origin_ref[1], coord[0], coord[1]) > radius_nm:
            break
        points.append((fix, coord))
    return points


def _nearest_airway_nodes(lat, lon, limit_nm=ENDPOINT_SEARCH_NM):
    """
    找机场附近的航路点, 用于程序端点不在航路图上时接入航路网.
    逐个遍历航路图节点, 返回 [(距离, 名称)] 由近到远排序.
    """
    graph = _get_airways()
    ref = (lat, lon)                              # 参考点: 机场坐标
    result = []
    for node in graph:
        coord = _node_coord(node, near=ref)       # 带参考点, 否则同名点会散落到别的大洲
        if coord is None:
            continue                                  # 没坐标的节点无法计算距离
        d = _nm_distance(lat, lon, coord[0], coord[1])
        if d <= limit_nm:
            result.append((d, node))
    result.sort()
    return result


def _reach_map(graph, target, limit_nm=None, anchor=None):
    """
    反向可达性搜索: 返回 {节点: 到 target 的最短距离} (沿反向边扩展).

    用途: 程序端点常常不在航路图上, 需要就近接入. 但导航数据被切成几十个
    连通分量, 最近的那个点未必能到达目标, 逐个 A* 试代价太高(全图 3.4 万节点).
    这里从目标出发一次反向 Dijkstra, 就能知道哪些节点可以连通目标.
    limit_nm 限制扩展半径, 避免为了接一个点而跨越整个大陆.
    """
    # 半径默认取"起点到终点直线距离的 1.6 倍", 保证远端也能被覆盖;
    # 固定半径(如 700nm)会让北京->广州这类长航线一端落在半径外, 导致整组候选作废.
    if limit_nm is None:
        limit_nm = MAX_REACH_NM
    # 锚点量化到 0.1 度: 同一目的地的不同航线可共用一次搜索结果
    anchor_key = (round(anchor[0], 1), round(anchor[1], 1)) if anchor else None
    cache_key = ("reach", target, round(limit_nm), anchor_key)
    cached = _CACHE.get(cache_key)
    if cached is not None:
        return cached                     # 同一出口在多个组合里会重复用到, 缓存避免重算

    # 目标点常是 3 字母导航台名(如 PUD/AND), 全球重名. 用 anchor(通常是目的地机场
    # 坐标)解析目标点, 否则会拿到另一个大洲的同名点, 整张可达图全部跑偏.
    # 注意: 不能用"上一个节点的坐标"作参考, 那会让坐标逐步漂移(伦敦 -> 白俄 -> 中国).
    t_coord = _node_coord(target, near=anchor)
    # 用"无向视图"反向扩展: 导航数据里大量航路是单向的(AND 在图上没有任何入边),
    # 若只用反图, 双向可走的真实连接会被漏掉. 可达性判断只需要"连通"即可.
    rev = _undirected_edges()
    dist = {target: 0.0}
    heap = [(0.0, target)]
    coord_cache = {target: t_coord}

    def coord_of(node, ref=None):
        """坐标一律以目标点 t_coord 为参考, 避免沿搜索路径逐步漂移到别的大洲."""
        if node in coord_cache:
            return coord_cache[node]
        c = _node_coord(node, near=t_coord)
        coord_cache[node] = c
        return c

    while heap:
        d, node = heapq.heappop(heap)
        if d > dist.get(node, float("inf")):
            continue
        if d > limit_nm:
            continue
        nc = coord_of(node, t_coord)
        for nb in rev.get(node, ()):
            cb = coord_of(nb, nc or t_coord)
            if nc and cb:
                step = _nm_distance(nc[0], nc[1], cb[0], cb[1])
            else:
                step = 100.0
            nd = d + step
            if nd < dist.get(nb, float("inf")) and nd <= limit_nm:
                dist[nb] = nd
                heapq.heappush(heap, (nd, nb))
    _CACHE[cache_key] = dist
    return dist


def _dijkstra(graph, source, target, reverse_penalty=0.0, target_ref=None):
    """
    在航路网络上找 source -> target 的最短路径 (按大圆距离加权)。

    导航数据的航路大量是"单向"的(Jeppesen 只记录实际发布方向), 若严格按
    有向图搜索, 北京<->上海这类对向航路会断掉。因此:
      - reverse_penalty=0    : 允许逆行(当作双向航路), 只做距离最优, 用于兜底;
      - reverse_penalty>0    : 边代价 = 距离 * (1 + reverse_penalty), 优先走顺向
                               (单向航路本来就是给某个方向的, 顺向才是合规航路).

    返回 (点列表, 逆行边数); 不连通返回 (None, 0)。
    """
    if source == target:
        return [source], 0
    if not graph.get(source) or not graph.get(target):
        return None, 0
    # 目标点必须按"目的地机场位置"解析: DANDE 在 RJ(日本)和 TJ(加勒比)各有一个同名点,
    # 无参考解析取到加勒比那个后, A* 的启发式把目标估在 1 万海里外, 退化为全图搜索,
    # 单次查询从几十毫秒膨胀到 4 秒以上.
    t_coord = _node_coord(target, near=target_ref)

    # 坐标缓存: 值同时记下"解析时用的参考点". 同名导航台(VOR/NDB)全球重名,
    # 同一个节点名在不同区域可能对应不同实体, 若只按名字缓存, 后面的同名点
    # 会复用远处解析出来的坐标, 航路就一步步漂到别的大洲(LBN 曾落到白俄罗斯).
    coord_cache = {}                      # {节点: (坐标, 参考点)}

    def coord_of(node, ref=None):
        """取节点坐标(带缓存). 仅当缓存时的参考点足够接近时才复用."""
        hit = coord_cache.get(node)
        if hit is not None:
            cached_coord, cached_ref = hit
            if ref is None or cached_ref is None:
                return cached_coord
            if _nm_distance(cached_ref[0], cached_ref[1], ref[0], ref[1]) <= MAX_FIX_JUMP_NM:
                return cached_coord           # 参考点相近, 复用之
        c = _node_coord(node, near=ref)
        if c is None:                          # 带参考点失败时退回无参考点解析
            c = _node_coord(node)
        if c is not None:
            coord_cache[node] = (c, ref)
        return c

    src_coord = coord_of(source)

    def edge_cost(a, b):
        """a->b 的距离代价; 无坐标时用常数 100 海里保证搜索继续进行。"""
        ca = coord_of(a, src_coord)
        cb = coord_of(b, ca or src_coord)
        if ca and cb:
            return _nm_distance(ca[0], ca[1], cb[0], cb[1])
        return 100.0

    def heuristic(n):
        """A* 启发式: 节点到目标的直线距离, 可采纳, 保证结果最优。"""
        nc = coord_of(n, src_coord)
        if nc and t_coord:
            return _nm_distance(nc[0], nc[1], t_coord[0], t_coord[1])
        return 0.0

    open_heap = [(heuristic(source), 0.0, source, 0)]   # (f, g, 节点, 逆行次数)
    best_g = {source: 0.0}
    prev = {source: None}
    # 反图只取一次: 原来在每轮内层循环里调用 _reverse_edges(), 每个邻居都要
    # 查一次缓存字典并重建两个列表, 单次查询能因此慢到 4 秒以上.
    rev_graph = _reverse_edges() if reverse_penalty > 0 else {}
    while open_heap:
        _, g, node, rev = heapq.heappop(open_heap)
        if node == target:                              # 出堆即最优
            path = []
            while node is not None:
                path.append(node)
                node = prev[node]
            return path[::-1], rev
        if g > best_g.get(node, float("inf")):
            continue
        # 正向邻居 + 逆向邻居(记录哪些是逆着航路方向走的)
        successors = [(n, False) for n in graph.get(node, ())]
        rev_nbrs = rev_graph.get(node)
        if rev_nbrs:
            successors.extend((n, True) for n in rev_nbrs)
        for nbr, is_reverse in successors:
            step = edge_cost(node, nbr)
            if is_reverse:
                step *= (1.0 + reverse_penalty)         # 逆行加罚, 让搜索优先顺向
            ng = g + step
            if ng >= best_g.get(nbr, float("inf")):
                continue
            best_g[nbr] = ng
            prev[nbr] = node
            heapq.heappush(open_heap, (ng + heuristic(nbr), ng, nbr, rev + (1 if is_reverse else 0)))
    return None, 0


def _reverse_edges():
    """构建反向边表 {节点: {能从它反向到达的点}}, 用于允许航路逆行。"""
    rev = _CACHE.get("reverse_edges")
    if rev is None:
        rev = {}
        for a, nbrs in _get_airways().items():
            for b in nbrs:
                rev.setdefault(b, set()).add(a)
        _CACHE["reverse_edges"] = rev
    return rev


def _undirected_edges():
    """构建无向邻接表 {节点: set(邻居)}, 忽略航路方向, 只用于连通性判断."""
    cached = _CACHE.get("undirected_edges")
    if cached is not None:
        return cached
    und = {}
    for a, nbrs in _get_airways().items():
        for b in nbrs:
            und.setdefault(a, set()).add(b)
            und.setdefault(b, set()).add(a)
    _CACHE["undirected_edges"] = und
    return und


def _dedupe_consecutive(points):
    """去掉列表中相邻重复项 (SID/STAR 与中转航路点在终点处会重叠)."""
    out = []
    for p in points:
        if not out or out[-1] != p:
            out.append(p)
    return out


# ============================ 候选程序选择 ============================


def _score_procedure_pair(sid_pts, star_pts, dep_coord, arr_coord):
    """
    给一对 (SID, STAR) 做几何预检, 不合理返回 None.

    程序端点已由 _pick_candidates 按方位筛选过, 这里只做物理合理性检查:
    交接点(SID 终点)必须离起飞机场足够远, 接收点(STAR 起点)必须离降落机场足够远,
    否则 "SID 刚起飞就交接" 或 "STAR 起点就在机场" 的组合会被选中, 产生虚假中转点.
    """
    if not sid_pts or not star_pts:
        return None
    sid_end = sid_pts[-1][1]                          # SID 最后一个点 = 交给航路的点
    star_start = star_pts[0][1]                       # STAR 第一个点 = 从航路接收的点
    if _nm_distance(dep_coord[0], dep_coord[1], sid_end[0], sid_end[1]) < 5.0:
        return None                                   # 交接点几乎就在起飞机场
    if _nm_distance(arr_coord[0], arr_coord[1], star_start[0], star_start[1]) < 5.0:
        return None                                   # 接收点几乎就在降落机场
    return 0.0


def _is_monotonic(pts, airport_coord, is_sid):
    """
    判断程序的航段距离是否单调变化:
      SID 离机场应逐段变远(允许 15 海里的抖动), STAR 应逐段变近.
    折返型程序(入口点 -> 远离 -> 回来)会在这一步被判为非单调而被降权.
    """
    dists = [_nm_distance(airport_coord[0], airport_coord[1], c[0], c[1]) for _, c in pts]
    if len(dists) < 3:
        return True                                    # 段数太少, 不做判断
    violations = 0
    for prev, cur in zip(dists, dists[1:]):
        if is_sid and cur < prev - 15.0:               # SID 反而靠近机场
            violations += 1
        if (not is_sid) and cur > prev + 15.0:         # STAR 反而远离机场
            violations += 1
    return violations == 0


def _pick_candidates(procedures, airport_coord, other_coord, navaids, is_sid,
                     airport=None, max_procs=12):
    """
    从某机场的全部程序里挑出可用的候选: [(程序名, 点列表, 端点), ...]
    is_sid=True 取最后一个点作为端点(交给航路), False 取第一个点(从航路接收).
    按"端点是否朝目的地、是否靠近机场"排序并截断到 max_procs 个.
    """
    scored = []
    # 同一程序名可能有多个独立块(不同跑道/过渡段), 每块单独评估,
    # 否则拼起来的点列会来回跳动, 被单调性检查误杀.
    for name, blocks in procedures.items():
      for info in blocks:
          pts = _procedure_points(info["rows"], name, navaids, airport, PROCEDURE_RADIUS_NM)
          if len(pts) < 2:
              continue                                  # 少于两个点的程序无意义
          # 整段程序必须紧贴本机场, 否则是残缺/错位的程序, 直接丢弃
          if max(_nm_distance(airport_coord[0], airport_coord[1], c[0], c[1])
                 for _, c in pts) > PROCEDURE_RADIUS_NM:
              continue
          end = pts[-1] if is_sid else pts[0]
          end_fix, end_coord = end
          # 起降机场本身的 VOR/NDB 会作为程序第一点出现(如 JFK、LAX),
          # 它等同于机场, 不能算 SID 终点或 STAR 起点, 否则会出现 "KJFK, JFK, ..."
          if end_fix in _get_airports():
              continue
          # 端点必须在航路图上, 且该名字在航路图里确实是"本机场附近"的那个点.
          # 反例: EGLL 的 SID 终点 LON 与英国导航台同名, 但 earth_awy.dat 里的 LON
          # 属于 ZG(中国)区域, 拿它当起点会把航路引到中国去.
          on_airway = bool(_get_airways().get(end_fix))
          if on_airway:
              gw = _node_coord(end_fix, near=end_coord)   # 按程序端点坐标解析航路图上的同名点
              if gw is None or _nm_distance(gw[0], gw[1], end_coord[0], end_coord[1]) > MAX_FIX_JUMP_NM:
                  on_airway = False                       # 图上同名点离得太远, 视为不接入
          d_to_origin = _nm_distance(airport_coord[0], airport_coord[1],
                                     end_coord[0], end_coord[1])   # 端点离开本场的距离
          # 端点应位于"朝另一机场那一侧": 方位偏差越小越好.
          #   SID: 从起飞机场指向 SID 终点, 应与"起飞->目的地"同向;
          #   STAR: 从降落机场指向 STAR 入口, 同样应与"目的地->起飞"同向
          #        (入口在来向一侧). 两处都用 "本机场 -> 端点" 的方位,
          #        之前 STAR 误用 "端点 -> 本机场", 会得到相差 180 度的反结论,
          #        导致 LFPG 的 STAR 全被当成"背向"筛掉.
          if is_sid:
              want = _bearing(airport_coord[0], airport_coord[1], other_coord[0], other_coord[1])
          else:
              want = _bearing(airport_coord[0], airport_coord[1], other_coord[0], other_coord[1])
          got = _bearing(airport_coord[0], airport_coord[1], end_coord[0], end_coord[1])
          dev = _angle_diff(got, want)
          # 方位判据只对"远端交接点"有意义: 端点离机场很近时(成田 STAR 入口离场仅 13nm),
          # 一点位置误差就能让方位偏百来度, 此时方位不作为筛选条件.
          if dev > BEARING_WINDOW_DEG and d_to_origin > BEARING_MIN_DIST_NM:
              continue                                  # 方位背离过大, 不是本航段的方向
          # 单调性: STAR 应一路靠近降落机场, SID 应一路远离起飞机场.
          # 用来排除 "先到入口点、再折回来" 的残缺程序(如 AND01A 只有 AND->D202X 两段).
          # 折返检查: STAR 末点(接进跑道那一点)之后不应再远离机场,
          # AND01A 这类 "AND -> D202X" 两段程序会在这里被直接淘汰
          if not is_sid and len(pts) >= 2:
              d_last = _nm_distance(airport_coord[0], airport_coord[1], pts[-1][1][0], pts[-1][1][1])
              d_prev = _nm_distance(airport_coord[0], airport_coord[1], pts[-2][1][0], pts[-2][1][1])
              if d_last > d_prev + 15.0:
                  continue                               # 末段在远离机场, 程序残缺
          # 完整性判据: STAR 必须一路向机场收敛, 末点不能又跑远.
          #   - 末点离机场超过 PROCEDURE_END_NM(45nm) -> CIFP 截断, 淘汰;
          #   - 末点比倒数第二点更远离机场 -> 程序在中途断掉(如 AND01A: AND->D202X),
          #     后面接不上跑道, 淘汰.
          # SID 末点本身就是远处的交接点, 两条都不适用.
          if not is_sid:
              end_dist = _nm_distance(airport_coord[0], airport_coord[1], pts[-1][1][0], pts[-1][1][1])
              if end_dist > PROCEDURE_END_NM:
                  continue
              if len(pts) >= 3:
                  prev_dist = _nm_distance(airport_coord[0], airport_coord[1], pts[-2][1][0], pts[-2][1][1])
                  if end_dist > prev_dist:
                      continue                          # 末点在远离机场, 程序残缺
          mono = 0 if _is_monotonic(pts, airport_coord, is_sid) else 1
          # 排序键: 方位偏差 -> 单调性 -> 是否在航路网上(0=在) -> 航段数多优先 -> 端点距离
          key = (dev, mono, 0 if on_airway else 1, -len(pts), d_to_origin)
          scored.append((key, name, pts, end_fix, on_airway))
    scored.sort(key=lambda x: x[0])
    return [(name, pts, end_fix) for _, name, pts, end_fix, _ in scored[:max_procs]]


# ============================ 主函数 ============================


def find_route(origin, destination, max_pairs=40):
    """
    规划 O-D 航路.

    参数:
        origin      起飞机场 ICAO, 如 "ZBAA"
        destination 降落机场 ICAO, 如 "ZSPD"
        max_pairs   最多尝试的 (SID, STAR) 组合数, 控制搜索成本

    返回:
        list[str], 结构为
            [起飞机场, SID终点, 中转航路点..., STAR起点, 降落机场]
        即 index 1 是 SID 终点, index -2 是 STAR 起点.
        找不到航路时返回 [origin, destination] 或抛 ValueError.
    """
    origin = origin.strip().upper()
    destination = destination.strip().upper()
    airports = _get_airports()
    navaids = _get_navaids()
    graph = _get_airways()

    if origin not in airports:
        raise ValueError(f"未知机场: {origin}")
    if destination not in airports:
        raise ValueError(f"未知机场: {destination}")
    if origin == destination:
        raise ValueError("起点和终点相同")

    dep_coord = airports[origin]
    arr_coord = airports[destination]

    # --- 1. 读两端 CIFP, 分组为程序 ---
    dep_sids = _group_procedures(_read_cifp_rows(origin)[0])
    arr_stars = _group_procedures(_read_cifp_rows(destination)[1])

    # --- 2. 生成候选 ---
    sid_cands = _pick_candidates(dep_sids, dep_coord, arr_coord, navaids, True, origin)
    star_cands = _pick_candidates(arr_stars, arr_coord, dep_coord, navaids, False, destination)

    best = None                                          # 最优组合 (score, 点列, 程序信息)
    lenient = None                                       # (绕飞比, 点列) 绕飞最小者, 硬约束全灭时降级用

    # --- 3. 组合搜索: SID 端点连着中转航路, 再接 STAR 起点 ---
    tried = 0
    for sid_name, sid_pts, sid_end in sid_cands:
        for star_name, star_pts, star_start in star_cands:
            if tried >= max_pairs:
                break
            tried += 1
            pre = _score_procedure_pair(sid_pts, star_pts, dep_coord, arr_coord)
            if pre is None:
                continue

            # 端点接入航路网: 端点在图上就直接用, 否则接附近的航路点.
            # 关键: 不能只试最近的一个点 —— 导航数据的连通分量有 34 个之多,
            # 最近的点可能落在对端不连通的分量里, 强行接过去会让 A* 沿整张图绕一大圈
            # (EDDF->LOWW 曾因此绕出 680nm, 直线只有 335nm).
            # 因此取附近若干个候选, 逐个尝试, 由最终的总航程决定用哪个.
            entry_opts = [(0.0, sid_end)] if graph.get(sid_end) else [
                (d, n) for d, n in _nearest_airway_nodes(*sid_pts[-1][1])[:ACCESS_CANDIDATES]
                if graph.get(n)
            ]
            exit_opts = [(0.0, star_start)] if graph.get(star_start) else [
                (d, n) for d, n in _nearest_airway_nodes(*star_pts[0][1])[:ACCESS_CANDIDATES]
                if graph.get(n)
            ]
            if not entry_opts or not exit_opts:
                continue

            # 选接入点: 先用一次"反向搜索"得出每个出口候选能连通哪些入口,
            # 避免对 6x6 个组合各跑一次全图 A*(那样单次查询要几十秒).
            # 反向搜索: 从出口候选出发沿入边扩展, 得到"能到达出口的节点集合"及其最短距离.
            reachable = {}
            for xd, xn in exit_opts[:2]:                   # 只试最近的 2 个出口, 覆盖绝大多数情况
                # 半径按航线长度自适应: 短程(如东京->大阪 250nm)不需要扫 3000nm 的图,
                # 否则单次搜索要扫全网点, 一次查询会拖到几十秒.
                leg_nm = _nm_distance(dep_coord[0], dep_coord[1], arr_coord[0], arr_coord[1])
                reach_nm = min(MAX_REACH_NM, leg_nm * 1.5 + 600.0)
                dists = _reach_map(graph, xn, limit_nm=reach_nm, anchor=arr_coord)
                for ed, en in entry_opts:
                    if en in dists:
                        total = ed + xd
                        key = dists[en] + total            # 航路代价 + 接入代价
                        if en not in reachable or key < reachable[en][0]:
                            reachable[en] = (key, xn, ed, xd)
            if not reachable:
                continue
            # 取总代价最小的 (入口, 出口) 对
            entry = min(reachable, key=lambda n: reachable[n][0])
            _, exit_node, entry_penalty, exit_penalty = reachable[entry]
            path, rev_count = _dijkstra(graph, entry, exit_node, REVERSE_PENALTY,
                                        target_ref=arr_coord)
            if not path:
                continue

            # --- 4. 拆分中转段 ---
            # path 两端分别是 SID 终点(或其接入点)与 STAR 起点(或其接入点).
            # 若两端恰好就是程序端点, 直接丢掉(程序段自带); 否则保留为接入点,
            # 这样 "SID终点 -> 接入点 -> 航路" 的连接关系不会断.
            middle = list(path)
            if middle and middle[0] == sid_end:
                middle.pop(0)                 # SID 终点已由 SID 段提供
            if middle and middle[-1] == star_start:
                middle.pop()                  # STAR 起点已由 STAR 段提供

            # --- 5. 拼完整序列 ---
            # 目标结构(只留端点):
            #   [起飞机场, SID终点, 中转航路点..., STAR起点, 降落机场]
            # SID/STAR 的中间程序点不展开, 只保留与航路衔接的那一个点.
            sid_exit = sid_pts[-1][0]                        # SID 终点 = 交给航路的点
            star_entry = star_pts[0][0]                      # STAR 起点 = 从航路接收的点

            # 中转段两端若与程序端点同名, 去掉重复(端点单独放)
            while middle and middle[0] == sid_exit:
                middle.pop(0)
            while middle and middle[-1] == star_entry:
                middle.pop()

            full = [origin, sid_exit] + middle + [star_entry, destination]
            full = _dedupe_consecutive(full)                 # 去掉相邻重复(如端点直接相连)

            # --- 结构校验: 长度足够, 且两个端点都在正确位置 ---
            if len(full) < 3 or full[1] != sid_exit or full[-2] != star_entry:
                continue

            # --- 6. 评分: 中转点越多越好(说明走了真实航路), 端点接入代价与绕飞越少越好 ---
            total_nm = 0.0
            detour = 0.0
            if len(full) >= 2:
                # 估算总航程 vs 直线距离, 作为绕飞惩罚
                # 逐点推进时把上一点的坐标当参考, 否则 PK/WOD 这类三字母台名
                # 会解析到另一个大洲, 总航程会被算成几千海里.
                prev_coord = airports.get(full[0]) or _node_coord(full[0])
                for i in range(1, len(full)):
                    cur = airports.get(full[i]) or _node_coord(full[i], near=prev_coord)
                    if prev_coord and cur:
                        total_nm += _nm_distance(prev_coord[0], prev_coord[1], cur[0], cur[1])
                    if cur:
                        prev_coord = cur
                direct = _nm_distance(dep_coord[0], dep_coord[1], arr_coord[0], arr_coord[1])
                detour = total_nm / max(direct, 1.0) - 1.0  # 0 = 完全直线
            # --- 纵向单调性: 沿大圆航线投影应基本递增, 大幅倒退说明绕了远路 ---
            # 例: EDDF->LOWW 曾有候选先飞到 18E 再折返, 投影一路冲过终点又退回来.
            projections = []
            _prev = dep_coord
            for _p in full[1:-1]:
                _c = airports.get(_p) or _node_coord(_p, near=_prev)
                if _c:
                    projections.append(_along_track(_c[0], _c[1], dep_coord[0], dep_coord[1],
                                                    arr_coord[0], arr_coord[1]))
                    _prev = _c
            backwards = sum(1 for i in range(1, len(projections))
                            if projections[i] < projections[i - 1] - 5.0)   # 容忍 5nm 抖动
            if projections:
                total_nm = max(total_nm, projections[-1])                # 投影里程作为下界
                if backwards > max(2, len(projections) // 4):
                    continue                             # 超过 1/4 的航段在倒退, 视为绕远

            # --- 绕飞硬约束: 绕飞比过大说明算法钻进了错误的航路分支, 直接淘汰 ---
            # 例: EDDF->LOWW 曾有组合先向南偏出 175nm 再折返, 总航程 602nm 对直线 335nm.
            direct = _nm_distance(dep_coord[0], dep_coord[1], arr_coord[0], arr_coord[1])
            detour_ratio = total_nm / max(direct, 1.0)
            # 绕飞最小的组合单独留一份, 万一所有组合都超限(极端地理条件)还能降级返回.
            if lenient is None or detour_ratio < lenient[0]:
                lenient = (detour_ratio, full)
            if detour_ratio > MAX_DETOUR_RATIO:
                continue                              # 绕得离谱, 换下一组

            # 评分: 总航程 + 逆行惩罚 + 端点接入代价; 超软阈值的绕飞按比例重罚.
            detour_penalty = max(0.0, detour_ratio - DETOUR_SOFT_RATIO) * total_nm * 3.0
            score = total_nm + 60.0 * rev_count + entry_penalty + exit_penalty + detour_penalty

            if best is None or score < best[0]:
                best = (score, full, (sid_name, star_name, len(middle)))
        if tried >= max_pairs:
            break

    # --- 7. 兜底 ---
    # 7a. 有候选但全部超过绕飞上限: 返回绕飞最小的那条, 至少保证航路真实可用.
    if best is None and lenient is not None:
        return lenient[1]
    # 7b. 完全没有可用 SID/STAR 组合: 机场 -> 最近航路点 -> 航路 -> 最近航路点 -> 机场.
    if best is None:
        # 只挑真正有出边的节点, 否则会把单向航路终点当入口
        near_dep = [(d, n) for d, n in _nearest_airway_nodes(*dep_coord) if graph.get(n)]
        near_arr = [(d, n) for d, n in _nearest_airway_nodes(*arr_coord) if graph.get(n)]
        if not near_dep or not near_arr:
            return [origin, destination]                # 机场附近没有任何航路
        path, _ = _dijkstra(graph, near_dep[0][1], near_arr[0][1], REVERSE_PENALTY)
        if not path:
            return [origin, destination]
        return _dedupe_consecutive([origin] + path + [destination])

    return best[1]


# ============================ 航路名展开 (可选辅助) ============================


def route_with_airways(route):
    """
    把点列表转换成 [(点, 航路名)] 形式, 航路名取该航段在 earth_awy.dat 中的第一条.
    机场与程序段的航段标记为 "SID"/"STAR"/"-".
    """
    graph = _get_airways()
    result = []
    for i, node in enumerate(route):
        if i == 0:
            result.append((node, "-"))                  # 起点机场
            continue
        prev = route[i - 1]
        names = graph.get(prev, {}).get(node)
        result.append((node, names[0][0] if names else "-"))
    return result


# ============================ 命令行自测 ============================

if __name__ == "__main__":
    import sys
    import time

    dep = sys.argv[1] if len(sys.argv) > 1 else "EDDF"
    arr = sys.argv[2] if len(sys.argv) > 2 else "EGLL"
    t0 = time.time()
    r = find_route(dep, arr)
    print(f"{dep} -> {arr}  ({len(r)} 个点, {time.time() - t0:.2f}s)")
    # 不要直接 print(r): list 的 repr 会给每个字符串加单引号.
    # 用 join 拼成字符串输出, 首尾的方括号也一起去掉, 读起来就是一条正常航路.
    print(" ".join(r))
    if len(r) >= 3:
        print("SID 终点:", r[1], "| STAR 起点:", r[-2])
    print("带航路名:", route_with_airways(r))
