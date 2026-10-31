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

# --- 航路点稀疏化 (MCDU 输入友好) ---
# 原始 earth_awy.dat 的航路是逐段短边(5-40nm), 直接输出会得到几十个相邻点,
# 在 MCDU 里要敲很久. 这里在搜索阶段就把导航图压缩成"跳跃图":
# 同一条航路上相距 SPARSE_MIN_NM..SPARSE_MAX_NM 的两个点直接连一条边,
# 路径自然只剩稀疏的关键点. 上限 200nm 是因为导航数据库里有的航段本身就有 100-190nm.
SPARSE_MIN_NM = 70.0         # 相邻输出点的最小间距: 比这更近的点在搜索里被跳过(除非被迫折返)
SPARSE_MAX_NM = 200.0        # 单次跳跃的最大间距: 超过必须落地, 否则直线化会偏离真实航路
SPARSE_RUN_LIMIT_NM = 1200.0 # 沿同一航路连续跳跃的累计上限, 之后必须落地
SPARSE_LOOKAHEAD = 64        # 构建跳跃边时每步最多访问的节点数
SPARSE_RETURN_FACTOR = 6.0   # 折返惩罚: 跳过去又沿原航路倒回来的额外代价 = 折返距离 x 此倍数
LATERAL_DETOUR_LIMIT = 1.75    # 大圆里程/沿航线投影里程 的上限, 超过说明横向绕了大圈
                             # (这条是防止"飞到另一半地球再折回来", 与正常的缓弯绕行无关)
SPARSE_MAX_DEVIATION_NM = 60.0   # 删点allowed的最大横向偏离: 超过就说明那是真实转弯点
FALLBACK_MAX_HOPS = 24       # 兜底路径允许的最大跳跃数, 超过改用逐边路径
SPARSE_MAX_FALLBACK_NM = 320.0  # 兜底路径的几何简化上限(机场间距可能很大), 仍不超航段本身长度
ACCESS_DIST_LIMIT_NM = 260.0 # 两端接入距离之和的上限, 超过说明程序端点离航路太远
MAX_DETOUR_RATIO = 1.70      # 绕飞比上限(总航程/大圆距离). 放宽的原因: 真实航路常常比
                             # 直线绕不少(EKCH->LOWW 全程走航路 1.62, 直线拼接只要 1.26),
                             # 硬卡 1.35 会把真实航路全毙掉, 只能退回不存在的直线航段.
                             # 真正的"折来折去"由下面的横向绕行/倒退/折返检查负责拦.
DETOUR_SOFT_RATIO = 1.25     # 评分时开始加罚的绕飞比. 抬到 1.25 是为了别把真实航路
                             # 罚到输给兜底直线; 超软阈值部分仍按里程 3 倍重罚, 选路依然偏向短而直的.

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
    dx, dy = (bx - ax) * math.cos(math.radians((lat1 + lat2) / 2.0)), by - ay
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
    entries = _get_fixes().get(name)
    if entries:
        return _pick_fix_entry(entries, None, near)
    # earth_fix 只收 ENRT 级航路点; 美国内陆的 VOR/NDB(MCB/MEI/MGM/RBV)只在
    # earth_nav 里, 不查这张表整条跨大洋航路会一个坐标都解析不出来,
    # 于是航迹被压成起降两点之间的一条直线.
    fs = _get_fixes().get(name)
    if fs and len(fs) == 1 and near:
        # 五字母航路点全球重名的概率极低, 只有一份就用它 —— 但必须先过"方向合理"
        # 这一关: NAKHA/MEOVA/ADBAG/GULOT 都只有一份坐标, 却在云南;
        # 光凭"唯一"就认, 伦敦出发的航路会一路串到中国再跳到百慕大.
        lat, lon, _, _ = fs[0]
        if _nm_distance(near[0], near[1], lat, lon) <= MAX_FIX_JUMP_NM:
            return (lat, lon)
    nav = _get_navaids().get(name)
    if nav:
        # earth_nav 里同一个台会出现多条( VOR、DME、ILS 各占一行, 坐标相同),
        # 不去重会把"同一个实体"误判成"多份同名点", 进而当成歧义丢掉.
        uniq = {(round(lat, 4), round(lon, 4)): (lat, lon, reg, reg) for lat, lon, reg in nav}
        picked = _pick_fix_entry(list(uniq.values()), None, near)
        if picked:
            return picked
    # earth_fix 里没有这个名字(三字母定位点常见): 用 CIFP 坐标表兜底, 同样按参考点消歧
    cifp = _get_cifp_coords().get(name)
    if cifp:
        return _pick_fix_entry(cifp, None, near)
    return None


def _pick_fix_entry(entries, airport=None, near=None):
    """在 [(lat, lon, 机场, 区域), ...] 里为同名点选一份: 本机场内 > 离参考点最近."""
    if airport:
        for lat, lon, apt, _ in entries:
            if apt == airport:
                return (lat, lon)                     # 程序所属机场的同名点最可信
    if near:
        # 只有一份坐标时没有歧义可言: 多远都得认. 跨洋航路的上一个点到下一个点
        # 动辄两千海里(EGLL 之后的 LCH/MCB/MEI 在美国东南), 用"离参考点 250nm"
        # 做上限会把整条大西洋航段的坐标全部判死, 航迹最后压成起降点之间一条直线.
        if len(entries) == 1:
            # 唯一一份也未必是本区域的: NAKHA/MEOVA/ADBAG/GULOT 全球只有一份,
            # 都在云南, 却出现在伦敦出发的航路里. 单点距离阈值会误杀跨洋航段
            # (EGLL 之后 3000nm 才到美国的点), 所以放宽为 MAX_REACH_NM 量级,
            # 只挡"跨了半个地球"的串味.
            lat, lon = entries[0][0], entries[0][1]
            if _nm_distance(near[0], near[1], lat, lon) <= MAX_REACH_NM:
                return (lat, lon)
        best = min(entries, key=lambda e: _nm_distance(near[0], near[1], e[0], e[1]))
        if _nm_distance(near[0], near[1], best[0], best[1]) > MAX_FIX_JUMP_NM:
            # 所有同名点都在几百海里外: 说明这个航路点在本区域没有唯一坐标,
            # 宁可返回 None 让调用方跳过, 也不能把航路连到别的大洲.
            return None
        return (best[0], best[1])
    lat, lon, _, _ = entries[0]
    return (lat, lon)


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
        # 只有一份坐标时没有歧义可言: 多远都得认. 跨洋航路的上一个点到下一个点
        # 动辄两千海里(EGLL 之后的 LCH/MCB/MEI 在美国东南), 用"离参考点 250nm"
        # 做上限会把整条大西洋航段的坐标全部判死, 航迹最后压成起降点之间一条直线.
        if len(entries) == 1:
            lat, lon, _, _ = entries[0]
            return (lat, lon)
        ranked = sorted(entries, key=lambda e: _nm_distance(near[0], near[1], e[0], e[1]))
        best = ranked[0]
        if _nm_distance(near[0], near[1], best[0], best[1]) <= MAX_FIX_JUMP_NM:
            return (best[0], best[1])
        # 有歧义且都不在附近: 宁可缺坐标(上层会跳过这一点), 也不猜一个可能是
        # 另一个大洲同名点的坐标.
        return None
    lat, lon, _, _ = entries[0]
    return (lat, lon)


def _load_airways():
    """
    解析 earth_awy.dat -> 双向邻接表 {航路点: {邻居: [(航路名, 最低FL, 最高FL), ...]}}
    行格式: 起点 区域 类型 终点 区域 类型 单双向(1单向/2双向) 基础FL 最高FL 航路名
    同一对点可能被多条航路连接, 所以邻居值用列表保存全部航路名.
    """
    graph = {}
    _load_airways.region = {}                     # {点: {区域: 次数}}, 识别同名异区实体
    with open(AWY_PATH, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.split()
            # 跳过文件头("I" 和 "1100 Version..." 等)以及字段不足的坏行
            if len(parts) < 11 or parts[0] == "I" or not parts[2].isdigit():
                continue
            _reg = _load_airways.region
            _reg.setdefault(parts[0], {})
            _reg[parts[0]][parts[1]] = _reg[parts[0]].get(parts[1], 0) + 1
            _reg.setdefault(parts[3], {})
            _reg[parts[3]][parts[4]] = _reg[parts[3]].get(parts[4], 0) + 1
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


def _load_cifp_coords():
    """
    无坐标名字的兜底表, 结果为空 — 保留接口以免调用方改动.

    背景: Navigraph 的 earth_fix.dat 只收录 ENRT 级航路点, 被 earth_awy.dat 引用的
    三字母定位点(INN/OB/PUD…)有些只出现在某个机场的 SID/STAR 里. 曾尝试从 CIFP
    文本行还原坐标, 但这份数据库(SID/STAR 行)并不带绝对坐标列 —— 只有跑道相对距离
    和 D 编码的磁航向/距离, 硬解出来的点会落到错误纬度. 因此这里不再猜坐标:
    解析失败的名字宁可按"无坐标"处理, 由逐边路径和绕飞评分兜底, 也好过把航路连到
    另一个大洲.
    """
    return {}


def _get_cifp_coords():
    """返回缓存的 CIFP 坐标表 {航路点: [(lat, lon, 机场), ...]}."""
    return _cached("cifp_coords", _load_cifp_coords)


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
        if d > limit_nm:
            continue
        # 名字对上了不代表就是本地那个实体: earth_awy 只有一个 "INN" 节点, 它的邻居
        # 是奥地利的 ADILO/BRENO/NORIN/RTT/TUNUM, 全部落在阿尔卑斯; 而上海浦东
        # 西南 1nm 处也有个 INN. 按坐标选点会选中这个"上海名字、奥地利身子"的节点,
        # 从它出发的每一跳都在 5000nm 外, 全被单跳上限剪掉 —— 入口点于是跳数为 0,
        # 搜索判定不连通, 直接返回只有起降两点的结果(ZSPD->ZBAA).
        # 用邻居的位置校验: 至少得有一个邻居落在本区域, 这个节点才算本地入口.
        if not _node_anchored(node, ref, coord):
            continue
        result.append((d, node))
    result.sort()
    return result


def _edge_cross_region(a, b):
    """
    这条边连接的两个点是否分属不同飞行区, 且其中至少一个是"多区域同名点".

    earth_awy 把同名实体并成了一个节点: MAPLE 在 VH(香港) 和 RJ(日本) 各有一个,
    文件里却写成 "MAPLE RJ -> RIPSI RJ" 的 Y121 —— 这个 MAPLE 是日本茨城附近的点,
    而香港赤鱲角旁边 10nm 处也有个 MAPLE. 按上下文解析时拿到的是香港那个,
    于是出现 MAPLE(香港) -> RIPSI(本州) 的 1561nm 单跳.
    """
    regs = getattr(_load_airways, "region", None)
    if not regs:
        _get_airways()
        regs = getattr(_load_airways, "region", {}) or {}
    ra, rb = regs.get(a) or {}, regs.get(b) or {}
    if not ra or not rb:
        return False
    if not (_multi_region(a) or _multi_region(b)):
        return False                                  # 两边都只有一个区, 不存在混淆
    # 多区域的那一端, 只要它在某个区里出现过、而另一端从不在那个区出现,
    # 这条边就有可能是跨区的假连接. 真正同一实体的边(两端区域完全一致)不受影响.
    for node, other in ((a, set(rb)), (b, set(ra))):
        if _multi_region(node) and set(regs[node]) - other:
            return True                               # 这个点在别的区也有一份, 会解析错
    return False


def _node_anchored(node, ref, coord):
    """节点的邻居里是否有落在 ref 附近的: 用来识破"名字本地、连接外地"的假节点."""
    graph = _get_airways()
    nbrs = list(graph.get(node, {}).keys()) + list(_reverse_edges().get(node, ()))
    if not nbrs:
        return False
    for nb in nbrs[:12]:
        c = _node_coord(nb, near=coord)
        if c and _nm_distance(coord[0], coord[1], c[0], c[1]) <= SPARSE_MAX_NM:
            return True                               # 有邻居在单跳范围内, 说明连接本身合理
    return False


def _multi_region(node):
    """
    这个名字是否在 earth_awy 里出现在多个飞行区域. MAPLE 同时属于 VH(香港)和 RJ(日本),
    是两个完全不同的实体共用了一个名字; INN 只属于 LO(奥地利), 上海附近那个 INN
    根本不在航路图里. 多区域的点不能按名字取坐标, 必须按上下文每次都重新定位.
    """
    regs = getattr(_load_airways, "region", None)
    if not regs:
        _get_airways()
        regs = getattr(_load_airways, "region", {}) or {}
    return len(regs.get(node) or ()) > 1


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
    if not graph.get(source):
        return None, 0
    # 目标允许"没有出边"(终端区末点只进不出, 如 REKLU), 但必须满足其一:
    #   a) 自己在图里;  b) 确实是某个图节点的邻居(入边存在).
    # 否则就是个纯同名噪声点(EUG/BOI/PIH 之类的美国台), 放进来会搜出一条
    # 跨半个地球的假航路(RJTT->RJBB 曾飞到美国西部).
    if not graph.get(target) and target not in _reverse_edges().get(target, set()) \
            and not any(target in (_get_airways().get(n) or {}) for n in _reverse_edges().get(target, ())):
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


# ============================ 航路点稀疏化 ============================
#
# 问题: earth_awy.dat 把一条航路拆成很多 5-40nm 的短边, 直接输出就是几十个相邻
# 同向航路点, 在 MCDU 里要一个个敲.
#
# 三层措施:
#   1. 搜索期压缩: _hop_neighbors 把"同一航路上 70-200nm 内的点"当成一跳,
#      A* 直接在跳跃图上跑, 得到的是长跳序列而不是逐边路径.
#   2. 折返防御: 跳跃只沿当前航路; 若跳过去的点之后被迫沿同一条航路倒回来,
#      _return_penalty 给该步重罚(不硬禁, 否则过顶/进出场侧的单向点会断链).
#   3. 事后简化: _sparsify_nodes 对最终点列做大圆几何简化, 去掉贴着直飞线的碎点
#      (如机场旁的 VOR).

_DEBUG = bool(os.environ.get("ROUTE_DEBUG"))
_ARROW = "\x00>"            # 超级边标记: "A\x00>B" 表示从 A 跳到了 B
_MAX_HOP_CACHE = {}          # {节点: [(邻居, 海里, 是否顺向)]}
_NAMED_COORD_CACHE = {}      # {节点: 坐标或 None}


def _named_coord(node):
    """
    无参考点的全局坐标, 结果缓存.

    跳跃图是脱离路径上下文构建的: 三字母导航台作为键时会重名(SFD 在英美各一个,
    HCH 在亚洲和美洲都有), 几十海里的航段会变成上万海里的大跳. 所以这里不走
    导航台表, 只认唯一性强的五字母航路点; 查不到就跳过, 由逐边路径兜底, 绝不猜.
    """
    if node in _NAMED_COORD_CACHE:
        return _NAMED_COORD_CACHE[node]
    if _multi_region(node):
        # 名字被多个飞行区共用(MAPLE 在 VH 和 RJ 各一个), 没有上下文就没有唯一答案.
        _NAMED_COORD_CACHE[node] = None
        return None
    entries = _get_fixes().get(node)
    if not entries:
        coord = None
    elif len(entries) == 1:
        coord = (entries[0][0], entries[0][1])
    else:
        # 同名多份(DANGO 在日本和墨西哥, BATTY 在阿拉斯加和比利时)时取第一条等于
        # 掷骰子, 会把一步 60nm 的航段变成 12000nm. 返回 None, 让调用方用上下文解析,
        # 实在解析不出就跳过这一点.
        coord = None
    _NAMED_COORD_CACHE[node] = coord
    return coord


def _airways_into():
    """反向航路表 {节点: {前驱: [(航路名, ...), ...]}}, 用于识别"沿原航路倒回来"."""
    c = _CACHE.get("airways_into")
    if c is not None:
        return c
    into = {}
    for a, nbrs in _get_airways().items():
        for b, names in nbrs.items():
            into.setdefault(b, {}).setdefault(a, []).extend(names)
    _CACHE["airways_into"] = into
    return into


def _direct_edges(node, coord, max_nm=SPARSE_MAX_NM):
    """
    直接相邻的航路点: [(邻居, 距离, 航路名集合, 邻居坐标)], 距离用 coord 现场算.

    必须用传入的本点坐标, 不能按名字查: BK/YEK/YZS/HCH/TAO 这类三字母点在 earth_fix
    里没有唯一坐标, 按名字取会拿到地球另一侧的同名台, 距离算成上万海里, 搜索随即乱跳
    (曾出现 RJTT->RJBB 绕到美国再飞回日本).
    """
    out = []
    graph = _get_airways()
    rev = _reverse_edges()
    for nb, entries in graph.get(node, {}).items():
        if _edge_cross_region(node, nb):
            continue                                  # 同名异区实体连成的假边, 不进跳图
        c = _node_coord(nb, near=coord)
        if coord and c:
            d = _nm_distance(coord[0], coord[1], c[0], c[1])
            # 相邻航路点不可能隔半个地球: 超过 max_nm 的"边"是数据里同名点被连错
            # (V172 的 CID->BEIGE: 航路记录指的是美国爱荷华州的 BEIGE, 而 earth_fix
            # 给 BEIGE 的坐标是大阪的进近点). 这种边必须剪掉, 不能进跳图.
            if d > max_nm:
                continue
        else:
            # 解不出坐标的邻居: 距离未知, 不能当成 40nm 的便宜边(日本->大阪会一路
            # 串过美国的 EUG/LTJ/IMB/BOI/PIH), 但也不能一刀剪断 —— 剪断之后航路图
            # 在跳跃层断连, 搜索绕地球找路(ZSPD->ZBAA 跑出 10776nm).
            # 给一个高于任何真边的代价: 只在确实没有带坐标的路可走时才会被选中,
            # 而且坐标未知的点不会被当作继续展开的跳板(见 _hop_neighbors 里的判断).
            d = max_nm * 1.0
        out.append((nb, d, {e[0] for e in entries}, c))
    for nb in rev.get(node, ()):
        if nb in graph.get(node, {}):
            continue                                  # 双向边已由上面的正向列表覆盖
        if _edge_cross_region(node, nb):
            continue
        c = _node_coord(nb, near=coord)
        if not (coord and c):
            d = max_nm * 1.0                          # 同上: 高代价而非剪断
        else:
            d = _nm_distance(coord[0], coord[1], c[0], c[1])
        if d > max_nm:
            continue
        out.append((nb, d, set(), c))
    return out


def _unit(lat1, lon1, lat2, lon2):
    """两点位移的单位方向向量, 用局部等距投影(经度乘 cos 纬度), 只用于比方向."""
    dy = lat2 - lat1
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return (0.0, 0.0)
    return (dx / norm, dy / norm)


def _reverse_run_nm(seq):
    """
    返回 (逆向里程, 总里程): 逐段位移与"起点->终点"总方向的点积为负的部分.

    只累计真正"往回飞"的里程, 而不是数点. SID 出港后转向、STAR 进场转弯都会让
    个别短段与总方向相反(几百米的转弯也算法向反向), 按点数统计会把正常航路
    整批毙掉 —— ZBAA->ZSPD 曾因此退化成一个空结果. 按里程算, 这类转弯只有
    几海里, 而 NAKHA(云南)串进伦敦->纽约的航路是上千海里, 一挡一个准.
    """
    if len(seq) < 2:
        return 0.0, 0.0
    bx, by = _unit(seq[0][0], seq[0][1], seq[-1][0], seq[-1][1])
    bad = total = 0.0
    for i in range(1, len(seq)):
        a, b = seq[i - 1], seq[i]
        d = _nm_distance(a[0], a[1], b[0], b[1])
        total += d
        sx, sy = _unit(a[0], a[1], b[0], b[1])
        if sx * bx + sy * by < -0.3:
            bad += d
    return bad, total


def _step_plausible(a, b, target, factor=2.5):
    """
    一步跳跃的物理合理性: 从 a 走到 b 之后, 到 target 的直线距离不应比从 a 出发时
    还远一半以上. 同名点(纽约的 MARIA vs 日本四国的 MARIA)按名字解析出的坐标
    会制造出上万海里的"跳", 这一步就能拦住, 不用等比 FlyBy 的绕飞比兜底.
    """
    if not (a and b and target):
        return True
    d0 = _nm_distance(a[0], a[1], target[0], target[1])
    d1 = _nm_distance(b[0], b[1], target[0], target[1])
    return d1 <= d0 * factor + 100.0


def _return_penalty(src, dst, src_coord=None, dst_coord=None, path_airways=()):
    """
    src->dst 是否属于"沿着刚过来时的那条航路又倒回去"; 返回额外代价(海里).

    只比对 path_airways(已走过的真实航路名)与 dst->src 的航路名, 不按名字查坐标,
    因此不会因为同名台把折返距离估错.
    """
    if not path_airways:
        return 0.0
    back = {e[0] for e in _get_airways().get(dst, {}).get(src, ())}
    # 折返判据: 来路与回路有共同航路名(沿原航路倒回), 或者来路里有逆向边
    # (path_airways 里为 None 表示"这条边是反着走的") —— 后者正是跨洲乱跳的典型形态.
    # 折返判据: 来路与回路有共同航路名(沿原航路倒回), 或者来路里有逆向边
    # (path_airways 里为 None 表示"这条边是反着走的") —— 后者正是跨洲乱跳的典型形态.
    if not (set(back) & {a for a in path_airways if a}) and None not in path_airways:
        return 0.0
    if src_coord and dst_coord:
        return _nm_distance(src_coord[0], src_coord[1], dst_coord[0], dst_coord[1]) * SPARSE_RETURN_FACTOR
    return 40.0 * SPARSE_RETURN_FACTOR


def _hop_neighbors(node, coord=None):
    """
    跳跃图的出边: [(落点, 沿航路里程, 航路名集合, 落点坐标)].

    规则: 从一个直接相邻的航路点出发, 沿"同一航路"(共用航路名的连续段)往前走,
    取 70-200nm 之间最远的一点作为落点; 该方向走不到 70nm 就退回走得最远的那点.
    只沿同一条航路推进, 所以不会出现"两条不同航路被当成一跳"的假连接
    (曾经 RJTT->RJBB 从东京直接跳到洛杉矶, 就是这种假边造成的).

    坐标一律用传入的 coord; 链上遇到没有唯一坐标的点就停在该方向, 绝不按名字猜.
    """
    if coord is None:
        coord = _node_coord(node)
    # 缓存键只带名字: 坐标按名字解析的结果在进程内是稳定的, 同一节点在不同路径上
    # 用不同参考点会解出同一个值(earth_fix 命中时), 带坐标反而让缓存几乎不命中,
    # 单次查询从 2 秒涨到 30 秒.
    cache_key = node
    cached = _MAX_HOP_CACHE.get(cache_key)
    if cached is not None:
        return cached
    out = []
    for first, d_first, names, c_first in _direct_edges(node, coord):
        if c_first is None or not names:
            # 落点没有坐标: 只能当作终点, 不能再从它往外跳. 否则 CID/DSM/IOW
            # 这种全球多名的点会连成一条横穿美国的走廊, 从东京一路串到纽约.
            out.append((first, max(d_first, 1.0), names, c_first))
            continue
        if d_first >= SPARSE_MIN_NM:
            out.append((first, d_first, names, c_first))
            continue
        # 头部太近: 继续沿同一航路向前找 70-200nm 的落点
        best = (first, d_first, c_first)
        queue = [(first, d_first, names, c_first)]
        seen = {first}
        steps = 0
        while queue and steps < SPARSE_LOOKAHEAD:
            cur, dist, cur_names, c_cur = queue.pop(0)
            steps += 1
            for nb, d, nnames, c_nb in _direct_edges(cur, c_cur):
                if nb in seen or c_nb is None:
                    continue
                shared = cur_names & nnames
                if not shared:
                    continue                     # 换了航路: 不能再算作同一跳
                nd = dist + d
                if nd > SPARSE_MAX_NM:
                    continue
                seen.add(nb)
                queue.append((nb, nd, shared, c_nb))
                if nd >= SPARSE_MIN_NM and nd > best[1]:
                    best = (nb, nd, c_nb)
        out.append((best[0], best[1], names, best[2]))
    # 同一落点只留最短的一条
    best_by_node = {}
    for nbn, dist, nm, c in out:
        if nbn == node:
            continue
        if nbn not in best_by_node or dist < best_by_node[nbn][1]:
            best_by_node[nbn] = (nbn, dist, nm, c)
    result = list(best_by_node.values())
    _MAX_HOP_CACHE[cache_key] = result
    return result


def _dijkstra_hop(graph, source, target, reverse_penalty=0.0, target_ref=None):
    """
    _dijkstra 的稀疏化版本, 在跳跃图上搜索.

    返回 (超级边序列, 逆行边数), 元素形如 "A\x00>B",
    用 _path_from_hop_path 展开成真实经过的航路点.
    """
    if source == target:
        return [source], 0
    if not graph.get(source):
        return None, 0
    # 目标允许"没有出边"(终端区末点只进不出, 如 REKLU), 但必须满足其一:
    #   a) 自己在图里;  b) 确实是某个图节点的邻居(入边存在).
    # 否则就是个纯同名噪声点(EUG/BOI/PIH 之类的美国台), 放进来会搜出一条
    # 跨半个地球的假航路(RJTT->RJBB 曾飞到美国西部).
    if not graph.get(target) and target not in _reverse_edges().get(target, set()) \
            and not any(target in (_get_airways().get(n) or {}) for n in _reverse_edges().get(target, ())):
        return None, 0
    # 目标点自己没坐标时(REKLU 这类终端区末点), 用 target_ref(通常是目的地机场)
    # 当几何参照. 不能让 t_coord 保持 None: _step_plausible 靠它拦跨洲同名点,
    # 一旦失效(RJTT->RJBB 的 STAR 起点 BEIGE 没坐标), 搜索就敢从东京一路
    # 走到美国西部的导航台, 单次查询从 1 秒变成 200 秒.
    t_coord = _node_coord(target, near=target_ref) or target_ref

    def heuristic(n):
        base = n.split(_ARROW)[-1]
        c = _node_coord(base, near=t_coord) or _node_coord(base)
        if c and t_coord:
            return _nm_distance(c[0], c[1], t_coord[0], t_coord[1])
        return 0.0

    # 状态带"当前点坐标"和"上一跳所在航路", 两者都不按名字重新解析.
    open_heap = [(heuristic(source), 0.0, source, source, 0)]
    best_g = {source: 0.0}
    prev = {source: None}
    seen_state = {}
    while open_heap:
        _, g, state, cur, rev = heapq.heappop(open_heap)
        if cur == target:                              # 出堆即最优
            hops = []
            while state is not None and state != source:
                hops.append(state)
                state = prev[state]
            return hops[::-1], rev
        if g > best_g.get(state, float("inf")):
            continue
        cur_coord, cur_names = seen_state.get(state, (_node_coord(cur), set()))
        for nb, dist, names, c_nb in _hop_neighbors(cur, cur_coord):
            nstate = cur + _ARROW + nb
            # 没有坐标的航路点不能当中转站: 它的距离未知(只能瞎估), 而且同名歧义
            # 无法消解. V172 的 CID->BEIGE 就是这种边 —— CID/DSM/IOW 全都没有坐标,
            # 于是从世界任何地方都能"穿过美国"接到大阪的进近点 BEIGE,
            # RJTT->RJBB 因此绕到美国再飞回来(单次查询 250 秒).
            # 允许它们作为落点(终点可能正好没坐标), 但不允许继续从它们往外跳.
            if cur_coord is not None and c_nb is None and nb != target:
                continue
            if c_nb is not None and not _step_plausible(cur_coord, c_nb, t_coord):
                continue                               # 坐标明显错位(同名点), 不许当跳
            forward_edge = bool(names & {e[0] for e in graph.get(cur, {}).get(nb, ())})
            step = dist
            if not forward_edge:
                step *= (1.0 + reverse_penalty)        # 逆行/反向边加罚
            step += _return_penalty(cur, nb, cur_coord, c_nb, cur_names)
            ng = g + step
            if ng >= best_g.get(nstate, float("inf")):
                continue
            best_g[nstate] = ng
            prev[nstate] = state
            seen_state[nstate] = (c_nb, names if forward_edge else set(names) | {None})
            heapq.heappush(open_heap, (ng + heuristic(nstate), ng, nstate, nb, rev + 1 if step > dist else rev))
    return None, 0


def _undo_backtrack(nodes):
    """
    撤销"走出去又顺着原路走回来"的折返段.

    数据里有这种链: W180 从 XOLAL 一路东去, 到 IDEPO/GLB 之后又冒出 XOLAL->MEKEP->...
    的重复节点(航路图把 ILS/LOC 附近误连成一串), 最短路径就先去走那段再折回来,
    输出就成了来回跑的路线.

    做法: 找到 nodes[i..j-1] 之后紧跟同一串逆序节点(走过又走回), 剪掉后半截;
    若回程只是部分重复(例如 ...PUDUG OS XOLAL MEKEP ... 又回到 UNRIX 附近),
    就退回"第一次经过该点的位置", 把中间那圈绕行整个删掉.
    """
    changed = True
    while changed and len(nodes) > 3:
        changed = False
        # 1) 完整逆序回折: A..B C D C B..  ->  ----
        for i in range(1, len(nodes) - 2):
            for j in range(i + 3, len(nodes) + 1):
                block = nodes[i:j]
                k = j + len(block) - 1
                if k >= len(nodes):
                    break
                if nodes[j:k + 1] == list(reversed(block)):
                    nodes = nodes[:i] + block + nodes[k + 1:]
                    changed = True
                    break
            if changed:
                break
        if changed:
            continue
        # 2) 回环: 某点再次出现, 且两次之间是一整圈 -> 砍掉后面那一圈
        seen = {}
        for idx in range(1, len(nodes)):
            p = nodes[idx]
            if p in seen:
                first = seen[p]
                if idx - first >= 3:
                    nodes = nodes[:first + 1] + nodes[idx + 1:]
                    changed = True
                    break
            else:
                seen[p] = idx
    return nodes


def _has_repeat(points):
    """
    点列里出现重复航路点(来回绕)则返回 True.
    只放过第一个点(起点本身), 末点必须查 —— 回折恰恰最容易发生在衔接待降落跑道时:
    ... GLAXY, SWAMP, RJAA 就是先到 SWAMP 再折回来, 但末点若是重复点就看不见.
    """
    seen = set()
    for p in points[1:]:
        if p in seen:
            return True
        seen.add(p)
    return False


def _point_to_segment_nm(p, a, b):
    """点 p 到线段 a-b 的近似垂直距离(海里), 局部平面近似."""
    k = math.cos(math.radians((a[0] + b[0]) / 2.0))
    ax, ay = a[1] * k, a[0]
    bx, by = b[1] * k, b[0]
    px, py = p[1] * k, p[0]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay) * 60.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy)) * 60.0


def _resolve_within(seq, name, max_seg_nm=400.0):
    """
    在坐标解析不出时, 用整条点列的上下文给 name 定位.

    同名航路点(BATTY 在阿拉斯加和比利时各一份)只靠名字会挑错洲, 但中转段的点
    一定落在前后两个已知点之间, 于是从两端各扫一遍: 正向扫描给出能连通的前缀,
    反向扫描给出后缀, 取夹在两者之间、且到两端距离不超过 max_seg_nm 的那一份.
    都不满足就返回 None —— 宁可删不动, 也不接受跨洲坐标.
    """
    entries = _get_fixes().get(name)
    if not entries:
        # 三字母台名(LNO/HUL/CIV/COA)只出现在 earth_nav 里, 同名现象同样严重,
        # 这里按整个点列里其它已知点取最近的一份, 而不是盲取第一条(那可能是非洲的台).
        navs = _get_navaids().get(name) or []
        refs = []
        for _q in seq:
            if _q == name:
                continue
            _c = _named_coord(_q)
            if _c:
                refs.append(_c)
        if not navs:
            return None
        if not refs:
            # 整条点列都没有可解析的锚点时无从消歧(YTE 在加拿大和日本各有一个),
            # 宁可返回 None 让这个点保持"未知", 也不猜一个可能跨洋的坐标.
            return None
        # 直接用"到最近参考点的距离", 不用包围盒中心: 中转段末尾那串台(UX/YFY/YTE/MQE)
        # 只和它前面的点有关, 用全体均值会被航路起点(上海)拖偏.
        best = None
        for e in navs:
            d = min(_nm_distance(e[0], e[1], r[0], r[1]) for r in refs)
            if best is None or d < best[0]:
                best = (d, (e[0], e[1]))
        # 允许跨段但不允许跨洲: 用 max_seg_nm 的 3 倍做上限
        return best[1] if best and best[0] <= max_seg_nm * 3.0 else None
    if len(entries) < 2:
        return (entries[0][0], entries[0][1])

    def _ok(c, r):
        return _nm_distance(c[0], c[1], r[0], r[1]) <= max_seg_nm

    idx = seq.index(name) if name in seq else -1
    before = after = None
    if idx > 0:
        r = None
        for _q in seq[:idx]:
            c = _node_coord(_q, near=r) if r else (_named_coord(_q) or _node_coord(_q))
            if c:
                r = c
        before = r
    if idx >= 0:
        r = None
        for _q in reversed(seq[idx + 1:]):
            c = _node_coord(_q, near=r) if r else (_named_coord(_q) or _node_coord(_q))
            if c:
                r = c
        after = r
    best = None
    for e in entries:
        c = (e[0], e[1])
        if before and not _ok(c, before):
            continue
        if after and not _ok(c, after):
            continue
        if before:
            d = _nm_distance(c[0], c[1], before[0], before[1])
            if best is None or d < best[0]:
                best = (d, c)
        else:
            return c
    if best:
        return best[1]
    if len(entries) == 1:
        return (entries[0][0], entries[0][1])
    return None


def _sparsify_nodes(nodes, min_nm=None, max_nm=None, max_dev_nm=30.0, path=None, dist_nm=None):
    """
    事后几何简化, 把"贴着直飞线的碎点"去掉, 让相邻点张力至少 min_nm.

    单遍贪心做不长(第二次跳点后偏离会超 max_dev_nm, 贪心就卡住), 所以反复扫,
    直到一趟没有删掉任何点为止. max_dev_nm 可以放宽到 max_nm/4: 只要被删的点
    仍然靠近新的直飞段, 航迹形状就没变, 只是中间那些 5-40nm 的碎点没了.

    有 path(逐段点列, 与 nodes 等长)时还会检查被删段确实在同一航路上 —— 否则
    会把真实的航路拐点抹掉, 变成"两点之间直飞"。
    """
    if len(nodes) < 3:
        return nodes
    min_nm = SPARSE_MIN_NM if min_nm is None else min_nm
    max_nm = SPARSE_MAX_NM if max_nm is None else max_nm
    # 偏离容差不能跟着 max_nm 一起放大: max_nm 只是"删掉后两个落点之间允许多远",
    # 而偏离是航迹形状. 容差过大会把转弯点当碎点删掉, 输出变成两点直飞
    # (曾出现 ZSPD->RJAA 只剩 11 个点, 最后一段 1004nm 直线穿海).
    max_dev_nm = max(max_dev_nm, min(max_nm / 4.0, SPARSE_MAX_DEVIATION_NM))
    coords = {}
    ref = None
    for n in nodes:
        # 统一用带参考解析: 三字母导航台(DBL/PUD)在全局表里可能取到别处的同名台,
        # 估高会让简化步骤把真实的转弯点当成"碎点"删掉.
        c = _node_coord(n, near=ref) if ref else (_named_coord(n) or _node_coord(n))
        if c is None and path:
            # 中转段的中间点常常在 earth_fix 里查不到(旧航路数据), 但整条完整点列
            # (含 SID/STAR 程序点)是逐点解析得来的, 从那里补坐标, 否则一个 None
            # 就把后面所有可合并的点全部冻住, 碎点一个都删不掉.
            _qc = _resolve_within(path, n)
            if _qc:
                c = _qc
        coords[n] = c
        if c:
            ref = c

    # 找 nodes[i] 在整条点列里的位置: 简化后中间的点可能已经不在 nodes 里了,
    # 所以必须查全序列, 不能拿"第 i 个"当对应关系.
    _full = list(path) if path else None
    _pos = {}
    if _full:
        for _i, _n in enumerate(_full):
            _pos.setdefault(_n, []).append(_i)

    def airborne(i, prev_n, cur_n):
        """prev_n -> cur_n 的航路名集合, 未知时返回 None(不做限制)."""
        if not _full:
            return None
        graph = _get_airways()
        names = set()
        for pi in _pos.get(prev_n, ()):
            for pj in _pos.get(cur_n, ()):
                if pj > pi:
                    names |= {e[0] for e in graph.get(_full[pi], {}).get(_full[pj], ())}
        return names or None

    # 无坐标的点(老数据里的三字母航路点)夹在两个已知点之间时, 按首尾等差数列估算
    # 位置. 只是用来判断"能不能合并", 不会进输出, 也不会跨洲乱跳: 首尾都是上下文
    # 里已确认的坐标, 插值点必然落在这两点连成的短段上.
    order = [n for n in nodes]
    i = 0
    while i < len(order):
        if coords.get(order[i]) is not None:
            i += 1
            continue
        j = i
        while j < len(order) and coords.get(order[j]) is None:
            j += 1
        a = coords.get(order[i - 1]) if i > 0 else None
        b = coords.get(order[j]) if j < len(order) else None
        if a and b and (j - i) <= 12:
            span = j - i + 1
            for k in range(i, j):
                t = (k - i + 1) / span
                coords[order[k]] = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        i = j

    # 上面只能处理"两端都有坐标"的空洞. 中转段末尾连着一串老式三字母航路点
    # (BK/CB/RB/UX/YFY...) 时, 空洞会一路顶到列表末尾, 这时改用最后一个已知点
    # 做锚, 沿前一段的方向等间距外推, 让这些点也能参与合并判断.
    last = None
    for _i in range(len(order) - 1, -1, -1):
        if coords.get(order[_i]) is not None:
            last = _i
            break
    if last is not None and last < len(order) - 1 and last > 0:
        ca, cb = coords.get(order[last - 1]), coords.get(order[last])
        if ca and cb:
            n = len(order) - 1 - last
            # 单侧外推只给很短的一段用: 后面若还有别的已知点(日本一侧的台),
            # 外推会把它越过, 反而把中间的点算到错误的位置上.
            if n <= 4:
                dlat = cb[0] - ca[0]
                dlon = cb[1] - ca[1]
                for k in range(1, n + 1):
                    coords[order[last + k]] = (cb[0] + dlat * k, cb[1] + dlon * k)

    # 剩下仍无坐标的(整段都没有锚点)按首尾线性插值补齐
    known = [i for i in range(len(order)) if coords.get(order[i]) is not None]
    if len(known) >= 2:
        for a_i, b_i in zip(known, known[1:]):
            if b_i - a_i <= 1:
                continue
            ca, cb = coords[order[a_i]], coords[order[b_i]]
            span = b_i - a_i
            for k in range(a_i + 1, b_i):
                t = (k - a_i) / span
                coords[order[k]] = (ca[0] + (cb[0] - ca[0]) * t, ca[1] + (cb[1] - ca[1]) * t)

    cur = list(nodes)
    changed = True
    while changed and len(cur) > 2:
        changed = False
        out = [cur[0]]
        i = 1
        while i < len(cur):
            prev = out[-1]
            nxt = cur[i]
            cp, cn = coords.get(prev), coords.get(nxt)
            if cp and cn and i + 1 < len(cur) and _nm_distance(cp[0], cp[1], cn[0], cn[1]) < min_nm:
                nxt2 = cur[i + 1]
                c2 = coords.get(nxt2)
                if c2:
                    d2 = _nm_distance(cp[0], cp[1], c2[0], c2[1])
                    dev = _point_to_segment_nm(cn, cp, c2)
                    # 几何偏离只是必要条件, 不能单独用它放行: 缓弯航路上被删点
                    # 离弦线可能只有 40nm, 但那一整段实际要走 142nm, 删掉后航迹
                    # 被压成直线, 里程凭空少掉 100nm(EKCH->LOWW 曾因此算出比大圆
                    # 还短的 0.89 倍). 所以再加一条"里程不许变短"的硬条件:
                    # 绕行里程 cp->cn->c2 必须不超过弦长 d2 的 2%, 否则那个点是
                    # 真实转弯点, 必须留在航迹里.
                    # 用"两站间距 + 小余量"而不是纯弦长: 航路点本来就落在航路边上,
                    # 相邻点之间的距离和必然略大于直线, 飞机也不会贴着弦飞.
                    # 3% 或 5nm 的余量足以让正常航段通过, 又拦得住把 80nm 弯道
                    # 压成 40nm 直线的删除.
                    detour_nm = (_nm_distance(cp[0], cp[1], cn[0], cn[1])
                                 + _nm_distance(cn[0], cn[1], c2[0], c2[1]))
                    straight_ok = detour_nm <= d2 * 1.03 + 5.0
                    if d2 <= max_nm and dev <= max_dev_nm and straight_ok:
                        i += 1                      # 丢掉 nxt, 换下一个当落点
                        changed = True
                        continue
            out.append(nxt)
            i += 1
        cur = out
    return cur


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
                if _DEBUG:
                    print("[gate] pre 淘汰: %s %s" % (sid_name, star_name))
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
                if _DEBUG:
                    print("[gate] 接入点缺失: %s %s" % (sid_name, star_name))
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
            # 在"跳跃图"上搜索: 中转段只保留 70nm 以上的关键点, 而不是每个航路短边都输出.
            # 先按"真实航路方向"搜(逆行加罚); 找不到再放开逆行重搜.
            # 原因: 导航库只记录实际发布方向, 很多终端区是单进口的漏斗 ——
            # 例如维也纳只能从东边的 OKR 进, 从丹麦方向严格顺向根本进不去,
            # 唯一"合法"的解是绕到东边多飞 270nm. 真实性优先于形式上的顺向.
            path, rev_count = _dijkstra_hop(graph, entry, exit_node, REVERSE_PENALTY,
                                            target_ref=arr_coord)
            if not path:
                # 只有"严格顺向根本走不通"时才放开逆行(维也纳这类单进口终端区).
                # 罚值不能接近 0: 0.05 等于不罚, 搜索会为了少走几个顺向边去绕
                # 一条 954nm 的跨区航路(RJTT->RJBB 因此飞到台湾再折回).
                path, rev_count = _dijkstra_hop(graph, entry, exit_node, 1.0,
                                                target_ref=arr_coord)
            if not path:
                if _DEBUG:
                    print("[gate] 跳跃搜索失败: %s %s" % (sid_name, star_name))
                continue

            # --- 4. 拆分中转段 ---
            # path 是跳跃序列 [ "A\x00>B", ... ], 末尾可能带终点(逐边模式)或全部是跳跃.
            # 还原起点必须是 sid_end: entry 只是航路网的接入点, 中转段要从程序终点接出去,
            # 两者之间那一点点距离由 [origin, sid_exit] + middle 自然覆盖.
            hops = [h for h in path if isinstance(h, str) and _ARROW in h]
            tail = [h for h in path if isinstance(p := h, str) and _ARROW not in h]
            if hops:
                flat = [sid_end] + hops + (tail[-1:] if tail else [])
            else:
                flat = [sid_end] + tail          # 逐边模式, path 已是完整点列
            # 展开时起点若不是该跳的真实起点(entry 与 sid_end 不同), 会把"接入点-程序终点"
            # 这一段也塞进第一跳, 于是整串标记原样返回. 那种情况改成从 entry 展开,
            # 再单独把 sid_end 补在最前面.
            m_start = entry if entry in (sid_end, flat[0]) or not hops else flat[0]
            middle = _path_from_hop_path([m_start] + flat[1:])
            if m_start != sid_end:
                middle = [sid_end] + middle
            # 任何内部符号泄漏到输出都说明拼接错位, 这种候选直接淘汰.
            if any((not isinstance(p, str)) or _ARROW in p for p in middle):
                if _DEBUG:
                    print("[skip] 拼接错位:", sid_name, star_name)
                continue

            # --- 5. 拼完整序列 ---
            # 目标结构(只留端点):
            #   [起飞机场, SID终点, 中转航路点..., STAR起点, 降落机场]
            sid_exit = sid_pts[-1][0]                        # SID 终点 = 交给航路的点
            star_entry = star_pts[0][0]                      # STAR 起点 = 从航路接收的点

            # 中转段两端与程序端点同名就剪掉(端点单独放)
            while middle and middle[0] == sid_exit:
                middle.pop(0)
            while middle and middle[-1] == star_entry:
                middle.pop()

            # 裁剪之后才判重复: 来回绕的航路(同一段走出去再走回来)在这里显形.
            if _has_repeat(middle):
                # 先试着剪掉"原路折返"那一段; 剪完还重复说明是真正的环线, 淘汰.
                middle = _undo_backtrack(middle)
            if _has_repeat(middle):
                if _DEBUG:
                    _seen = set()
                    _dup = [p for p in middle[1:] if p in _seen or _seen.add(p)]
                    print("[skip] 中转段重复: %s %s 重复点=%s" % (sid_name, star_name, _dup[:4]))
                continue

            full = [origin, sid_exit] + middle + [star_entry, destination]
            full = _dedupe_consecutive(full)                 # 去掉相邻重复(如端点直接相连)

            # 间距整形: MCDU 里逐点敲入 70 个点不现实, 把贴着航线的碎点合并掉.
            # 只在中转段里做(端点与程序衔接点必须保留), 且要求被删段同属一条航路.
            # 注意简化可能在衔接点旁边留下回折(如 ...GLAXY, SWAMP, RJAA), 所以整理完
            # 还要再查一遍重复 —— 回折的路线在 MCDU 里等于让飞机掉头.
            if len(full) > 4:
                head, inner, tailp = full[0], full[1:-1], full[-1]
                # 逐边(兜底)路径的里程上限里给 2x 余量, 让同一航路上更远的点也能落在
                # "被删掉也没关系"的判断范围内, 否则真实航路的细微弯曲会把点全保住.
                # 一律用 320nm 的上限做"能不能删"的判断, 实际落点仍受 70nm 下限约束,
                # 这样同一航路上的长弯(如 DBL 附近 14nm 的折点)也能被吞掉, 不会留下碎点.
                slim = _sparsify_nodes(inner, max_nm=SPARSE_MAX_FALLBACK_NM,
                                       max_dev_nm=SPARSE_MAX_FALLBACK_NM / 4.0, path=full)
                if len(slim) >= 2 or not inner:
                    full = _dedupe_consecutive([head] + slim + [tailp])
            if _has_repeat(full[1:-1]):
                if _DEBUG:
                    print("[skip] 整理后回折:", sid_name, star_name, len(full))
                continue

            # --- 结构校验: 长度足够, 且两个端点都在正确位置 ---
            if len(full) < 3 or full[1] != sid_exit or full[-2] != star_entry:
                if _DEBUG:
                    print("[gate] 结构 %d %s %s" % (len(full), sid_name, star_name))
                continue
            if any((not isinstance(p, str)) or _ARROW in p for p in full):
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
            # 投影点必须跳过解不出坐标的: 一旦某个点没坐标, 后面每个点都在错误的
            # 参考点下解析, 投影会集体塌到起点附近, 正常航路被判成横向绕行.
            projections = []
            _prev = dep_coord
            for _p in full[1:-1]:
                # 坐标缺失时必须跳过, 不能把 _prev 重置: 早先的写法让 _prev 停在
                # 第一个解不出坐标的点上, 后面所有点都在错误的参考点下解析,
                # 投影落到起点附近, 于是 70 点的正常航路被判成"横向绕行 4.69".
                _c = airports.get(_p) or _node_coord(_p, near=_prev)
                if _c:
                    projections.append(_along_track(_c[0], _c[1], dep_coord[0], dep_coord[1],
                                                    arr_coord[0], arr_coord[1]))
                    _prev = _c
            # 逆向航段: 已经解出坐标的点里, 如果某一步的位移与"起点->终点"的大圆
            # 方向基本相反, 说明航路在整体倒着走. NAKHA/MEOVA/ADBAG 这类云南点
            # 出现在伦敦->纽约的航路上时, 投影覆盖率不够会被上面的闸门放过,
            # 但"向东走"这件事本身就与目的地相反.
            _pts = []
            _pp = dep_coord
            for _p in full[1:-1]:
                _c = airports.get(_p) or _node_coord(_p, near=_pp)
                if _c:
                    _pts.append(_c)
                    _pp = _c
            _bad_nm, _tot_nm2 = _reverse_run_nm([dep_coord] + _pts + [arr_coord])
            if _tot_nm2 > 0 and _bad_nm > max(150.0, _tot_nm2 * 0.15):
                    if _DEBUG:
                        print("[gate] 逆向航段 %.0fnm/%.0fnm: %s %s"
                              % (_bad_nm, _tot_nm2, sid_name, star_name))
                    continue
            backwards = sum(1 for i in range(1, len(projections))
                            if projections[i] < projections[i - 1] - 5.0)   # 容忍 5nm 抖动
            if projections:
                total_nm = max(total_nm, projections[-1])                # 投影里程作为下界
                if backwards > max(2, len(projections) // 4):
                    if _DEBUG:
                        print("[gate] 倒退 %d: %s %s" % (backwards, sid_name, star_name))
                    continue                             # 超过 1/4 的航段在倒退, 视为绕远
                # 纵向单调还不够: 整段先绕去南半球再回来, 投影仍然一路"向前".
                # 用"实际大圆里程 / 投影里程"判断横向绕行, 正常航路一般 < 1.35.
                # 投影按"已知点的推进量"算; 中部一串点解不出坐标时 projections[-1]
                # 会停在中途, 比值虚高. 用"总里程 vs 直达 + 容忍"判横向绕行更稳.
                _lat_ratio = total_nm / projections[-1] if projections[-1] > 0 else 0.0
                _covered = projections[-1] / max(direct, 1.0)
                # 覆盖率不足 0.8 说明中部大量点没坐标, 投影里程不可信, 交给跨洲闸门与
                # 绕飞比把关; 覆盖良好时才用横向比 —— 否则正常航路会被误杀成绕行.
                if _covered >= 0.8 and _lat_ratio > LATERAL_DETOUR_LIMIT:
                    if _DEBUG:
                        print("[gate] 横向绕行 %.2f: %s %s"
                              % (_lat_ratio, sid_name, star_name))
                    continue

            # --- 绕飞硬约束: 绕飞比过大说明算法钻进了错误的航路分支, 直接淘汰 ---
            # 例: EDDF->LOWW 曾有组合先向南偏出 175nm 再折返, 总航程 602nm 对直线 335nm.
            direct = _nm_distance(dep_coord[0], dep_coord[1], arr_coord[0], arr_coord[1])
            detour_ratio = total_nm / max(direct, 1.0)
            # 绕飞最小的组合单独留一份, 万一所有组合都超限(极端地理条件)还能降级返回.
            if lenient is None or detour_ratio < lenient[0]:
                lenient = (detour_ratio, full)
            if detour_ratio > MAX_DETOUR_RATIO:
                if _DEBUG:
                    print("[gate] detour %.2f %s %s" % (detour_ratio, sid_name, star_name))
                continue                              # 绕得离谱, 换下一组

            # --- 合理性闸门: 中转段不能"凭空跳过半个地球" ---
            # 三字母点名全局重名(HCH/TAO/MARIA/CHILY...), 某些航路边上就会冒出一条
            # 直达另一大洲的假边. 表现是总航程远超直线, 但这些点没有坐标时绕飞比
            # 会算不出来, 于是必须用"中转段里程 vs 两端端点直线距离"兜一道.
            span_ok = True
            if middle:
                _m0 = sid_pts[-1][1]
                _m1 = star_pts[0][1]
                span_direct = _nm_distance(_m0[0], _m0[1], _m1[0], _m1[1])
                span_nm = 0.0
                _prev = _m0
                for _p in middle[1:] + [star_pts[0][0]]:
                    _c = airports.get(_p) or _node_coord(_p, near=_prev)
                    if _c:
                        span_nm += _nm_distance(_prev[0], _prev[1], _c[0], _c[1])
                        _prev = _c
                # 这一道只用来抓"凭空跨半个地球"的假边, 不负责判绕路远近:
                # 中转段两端直线很短(EKCH->LOWW 只有 340nm)时, 任何真实航路都会
                # 轻松超过 1.7, 于是好的 SID/STAR 组合被整批毙掉, 只剩兜底直线.
                # 跨洲假边的特征是"里程离谱"(上万海里), 用一个宽松得多、且带绝对
                # 余量的上限区分: 允许直达距离的 4 倍, 再加 800nm 给终端区的绕行.
                if span_direct > 1.0 and span_nm > span_direct * 4.0 + 800.0:
                    span_ok = False
            if not span_ok:
                if lenient is None:
                    lenient = (detour_ratio, full)
                if _DEBUG:
                    print("[gate] span 超限: %s %s" % (sid_name, star_name))
                continue

            # --- 跨洲保护: 相邻两点大圆距离超过 2000nm, 且不是南北向大圆航线 ---
            # 中纬度东西向转场不存在 2000nm 的相邻航段(最长的航线单段也就 1000nm 上下),
            # 这种跳跃几乎一定是同名点(HCH/TAO/ML 之类)解析到了别的大洲.
            # 南北向高纬航线(如跨极/跨洋)允许长段, 用"纬度变化超过经度变化"放行.
            # 跨度必须逐段检查, 不能只看总航程: 两个跨洲大跳可以互相抵消,
            # 总里程看着正常(曾出现 ZBAA->ZSPD 绕澳大利亚再飞日本的组合),
            # 所以首尾相接的"长段 + 长段"也要一并判死.
            hop_ok = True
            _prev = airports.get(full[0])
            _long_streak = 0
            # 用"两个已知点之间直接算距离"的中转段来判断跨洲, 而不是靠逐点 near 解析:
            # 逐点解析时同名点会被前一个点带偏, 长段变成"相邻短段", 检查等于失效.
            _seq = [[airports.get(full[0]), full[0]]]
            _pending = []
            for _p in full[1:]:
                # 这里的坐标只用来量跨度, 必须"全局无歧义"才好用: 带参考的解析会让
                # 同名点被前一个点带着走, 长段被切成一堆短段, 检查也就形同虚设.
                _c = airports.get(_p) or (_named_coord(_p) if _p in _get_fixes() else None)
                if _c is None:
                    _c = _resolve_within(full, _p)
                if _c:
                    _seq.append([_c, _p])
                else:
                    _pending.append(_p)
            for (_c0, _n0), (_c1, _n1) in zip(_seq, _seq[1:]):
                if not (_c0 and _c1):
                    continue
                seg = _nm_distance(_c0[0], _c0[1], _c1[0], _c1[1])
                if seg > 1500.0:
                    hop_ok = False
                    if _DEBUG:
                        print("[gate] 跨洲段 %s->%s %.0fnm (%s %s)"
                              % (_n0, _n1, seg, sid_name, star_name))
                    break
            for _p in full[1:]:
                _c = airports.get(_p) or _node_coord(_p, near=_prev)
                if _c and _prev:
                    seg = _nm_distance(_prev[0], _prev[1], _c[0], _c[1])
                    dlon = abs(_c[1] - _prev[1]) * 60.0 * math.cos(math.radians((_c[0] + _prev[0]) / 2.0))
                    dlat = abs(_c[0] - _prev[0]) * 60.0
                    if seg > 2000.0 and dlon > dlat:
                        hop_ok = False
                        break
                    if seg > 900.0:
                        _long_streak += 1
                        if _long_streak >= 2:
                            hop_ok = False
                            break
                    else:
                        _long_streak = 0
                if _c:
                    _prev = _c
            if not hop_ok:
                if _DEBUG:
                    print("[gate] hop 跨洲: %s %s" % (sid_name, star_name))
                continue

            # 评分: 总航程 + 逆行惩罚 + 端点接入代价; 超软阈值的绕飞按比例重罚.
            detour_penalty = max(0.0, detour_ratio - DETOUR_SOFT_RATIO) * total_nm * 3.0
            score = total_nm + 60.0 * rev_count + entry_penalty + exit_penalty + detour_penalty

            # 同一条走廊上的 (SID, STAR) 组合往往差不了几海里. 这种时候按点数挑,
            # 否则会选到总里程只短 0.1%、却多出十几个 5-20nm 碎点的组合 —— MCDU 里
            # 多敲十几个点没有任何好处. 只有明显更短(优 2.5% 以上)才压过点数.
            if best is None or score < best[0] * 0.975:
                best = (score, full, (sid_name, star_name, len(middle)))
            elif score <= best[0] * 1.025 and len(full) < len(best[1]):
                best = (score, full, (sid_name, star_name, len(middle)))
        if tried >= max_pairs:
            break

    # --- 7. 兜底 ---
    # 7a. 有候选但全部超过绕飞上限: 返回绕飞最小的那条, 至少保证航路真实可用.
    # 这条路径没走过候选阶段的去重/折返清理, 出口前补一遍, 否则会出现
    # "PAIRS 走了一遍又原路走回来"的双倍里程(曾经输出过 285 个点).
    if best is None and lenient is not None:
        # lenient 是"所有候选都被闸门毙掉后绕飞最小的一条", 它本身从没通过几何校验:
        # 曾出现 EGLL->KJFK 返回 LON NAKHA MEOVA ADBAG(云南) -> BUBSU -> DAL -> 纽约,
        # 绕飞 4.73 倍. 出口前再查一次逆向往返, 不合格就放弃, 交给 7b 的最近航路点兜底.
        _len_pts = []
        _lp = dep_coord
        for _p in lenient[1][1:-1]:
            _c = airports.get(_p) or _node_coord(_p, near=_lp)
            if _c:
                _len_pts.append(_c)
                _lp = _c
        if len(_len_pts) >= 2:
            _bad_nm, _tot_nm2 = _reverse_run_nm([dep_coord] + _len_pts + [arr_coord])
            if _tot_nm2 > 0 and _bad_nm > max(150.0, _tot_nm2 * 0.15):
                if _DEBUG:
                    print("[fallback] lenient 逆向 %.0fnm/%.0fnm, 放弃"
                          % (_bad_nm, _tot_nm2))
                lenient = None
    if best is None and lenient is not None:
        cand = _undo_backtrack(list(lenient[1]))
        if len(cand) > 4:
            head, inner, tailp = cand[0], cand[1:-1], cand[-1]
            inner = _sparsify_nodes(inner, max_nm=SPARSE_MAX_FALLBACK_NM,
                                    max_dev_nm=SPARSE_MAX_FALLBACK_NM / 4.0)
            cand = _dedupe_consecutive([head] + inner + [tailp])
        if _DEBUG:
            print("[fallback] lenient 路径 %d 点 -> %d 点" % (len(lenient[1]), len(cand)))
        return _sanitize_route(cand)
    # 7b. 完全没有可用 SID/STAR 组合: 机场 -> 最近航路点 -> 航路 -> 最近航路点 -> 机场.
    if best is None:
        if _DEBUG:
            print("[fallback] 无候选, 走最近航路点兜底")
        # 只挑真正有出边的节点, 否则会把单向航路终点当入口
        near_dep = [(d, n) for d, n in _nearest_airway_nodes(*dep_coord) if graph.get(n)]
        near_arr = [(d, n) for d, n in _nearest_airway_nodes(*arr_coord) if graph.get(n)]
        if not near_dep or not near_arr:
            return [origin, destination]                # 机场附近没有任何航路
        path, _ = _dijkstra_hop(graph, near_dep[0][1], near_arr[0][1], REVERSE_PENALTY,
                                target_ref=arr_coord)
        if not path or len(path) > FALLBACK_MAX_HOPS:
            # 跳数太多说明跳跃图在这里不连通(大量同名点解不出坐标), 退回逐边路径,
            # 之后由几何稀疏化把点列压到 70-200nm.
            path, _ = _dijkstra(graph, near_dep[0][1], near_arr[0][1], REVERSE_PENALTY)
        if not path:
            return [origin, destination]
        # 先展开成逐段真实航迹(每段带上真实航路名), 再做几何稀疏化
        detailed = _path_from_hop_path(path)
        if len(detailed) >= 2:
            detailed = _undo_backtrack(detailed)
            detailed = _sparsify_nodes(detailed, max_nm=SPARSE_MAX_FALLBACK_NM,
                                       max_dev_nm=SPARSE_MAX_FALLBACK_NM / 4.0)
        else:
            detailed = path
        # 兜底路径同样要过反向校验: 这一段没有经过候选循环的任何闸门,
        # EGLL->KJFK 曾在这里返回 LON NAKHA(云南) MEOVA ADBAG LXI ... DAL ...,
        # 绕飞 4.73 倍. 有逆向航段就放弃, 只给起降两点, 也好过给一条飞不出来的航路.
        _d_pts = []
        _dp = dep_coord
        for _p in detailed[1:-1]:
            _c = airports.get(_p) or _node_coord(_p, near=_dp)
            if _c:
                _d_pts.append(_c)
                _dp = _c
        if len(_d_pts) >= 2:
            _bad_nm, _tot_nm2 = _reverse_run_nm([dep_coord] + _d_pts + [arr_coord])
            if _tot_nm2 > 0 and _bad_nm > max(150.0, _tot_nm2 * 0.15):
                if _DEBUG:
                    print("[fallback] 最近航路点兜底逆向 %.0fnm/%.0fnm, 放弃"
                          % (_bad_nm, _tot_nm2))
                return _sanitize_route([origin, destination])
        return _sanitize_route(_dedupe_consecutive([origin] + detailed + [destination]))

    if _DEBUG:
        print("[selected] 正常候选 (%d 点) SID=%s STAR=%s" % (len(best[1]), best[2][0], best[2][1]))
    return _sanitize_route(best[1])


def _path_from_hop_path(start_and_hops):
    """
    把稀疏路径还原成逐段真实航迹: 对每一跳在两个落点之间跑一次逐边最短路,
    这样每一段都有真实航路名, 总航程/绕飞比也不会因为"拉直线"而失真.

    输入 [起点, "A\x00>B", ...] 或 [起点, B, C, ...] 均可.
    某一跳还原失败时直接落到该跳终点: 宁可少一个点, 也不把航路连错.
    """
    if not start_and_hops:
        return []
    graph = _get_airways()
    out = [start_and_hops[0]]
    cur = start_and_hops[0]
    for item in start_and_hops[1:]:
        if not isinstance(item, str):             # 防御: 非字符串项不参与拼接
            continue
        if _ARROW in item:
            a, b = item.split(_ARROW)
        else:
            a, b = cur, item
        if a != cur:                              # 边序列与本跳起点对不上, 直接跳过去
            out.append(b)
            cur = b
            continue
        # 逐边还原用"逆向也允许"的代价: 跳跃边本身就是按无向邻接生成的,
        # 严格按发布方向还原会把一条直跳拆成绕半个终端区的大弯
        # (BETUD->REKLU 因此从 510nm 变成 860nm).
        detail, _ = _dijkstra(graph, a, b, 1.0, target_ref=_node_coord(b))
        if detail:
            out.extend(p for p in detail[1:] if p != cur)
        else:
            out.append(b)
        cur = b
    return out


def _sanitize_route(route):
    """
    出口处的最后一道过滤: 任何非字符串或带内部跳跃标记的点都不许出现在结果里.
    正常拼接不会产生这种东西, 但兜底分支多, 出口统一挡一次最省事.

    顺便去掉"兜了一圈又回到同一个点"的非相邻重复: ZUUU->ZGGG 曾输出
    WFX BUVUR WFX BONPI, 飞机要在 WFX 上空绕一圈再走, MCDU 里就是一条回折航路.
    保留第一次出现, 后段整段重复的(真正闭合的环)才截掉.
    """
    out = [p for p in (route or []) if isinstance(p, str) and _ARROW not in p]
    if len(out) < 4:
        return out
    seen = {}
    for i, p in enumerate(out):
        if p in seen:
            # 回到已访问的点: 中间那段是绕出去又绕回来的环, 整段丢掉, 从第一次
            # 出现处直接接到 i+1, 再从头扫描(可能还有别的环).
            first = seen[p]
            return _sanitize_route(out[:first + 1] + out[i + 1:])
        seen[p] = i
    return out


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

    dep = sys.argv[1] if len(sys.argv) > 1 else "EKCH"
    arr = sys.argv[2] if len(sys.argv) > 2 else "LOWW"
    t0 = time.time()
    r = find_route(dep, arr)
    print(f"{dep} -> {arr}  ({len(r)} 个点, {time.time() - t0:.2f}s)")
    # 不要直接 print(r): list 的 repr 会给每个字符串加单引号.
    # 用 join 拼成字符串输出, 首尾的方括号也一起去掉, 读起来就是一条正常航路.
    print(" ".join(r))
    if len(r) >= 3:
        print("SID 终点:", r[1], "| STAR 起点:", r[-2])
    print("带航路名:", route_with_airways(r))
