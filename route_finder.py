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
    2. 解析两端 CIFP, 按程序名分组, 提取航段点列并校验(地名歧义/程序截断/折返)
    3. 选出候选 SID 与 STAR, 按方位角、程序完整性、单调性排序
    4. 对 (SID终点, STAR起点) 组合做 A* 最短路径搜索 (允许逆行但加罚)
    5. 按总航程 + 逆行代价评分, 取最优组合, 拼成 [机场, SID终点, 中转..., STAR起点, 机场]
"""

import heapq
import math
import os
import re

# ============================ 路径常量 ============================

# X-Plane 12 导航数据库根目录, 修改这里即可换到其他安装位置
XP_DATA_DIR = "/Users/aic06/X-Plane 12/Resources/default data"
CIFP_DIR = os.path.join(XP_DATA_DIR, "CIFP")          # 每个机场一个 .dat 文件
AWY_PATH = os.path.join(XP_DATA_DIR, "earth_awy.dat")  # 航路网络
FIX_PATH = os.path.join(XP_DATA_DIR, "earth_fix.dat")  # 航路点坐标
APT_PATH = os.path.join(XP_DATA_DIR, "earth_aptmeta.dat")  # 机场坐标
NAV_PATH = os.path.join(XP_DATA_DIR, "earth_nav.dat")  # 导航台(坐标兜底)

# ============================ 可调参数 ============================

ENDPOINT_SEARCH_NM = 150.0   # 程序端点不在航路图上时, 就近接入航路网的最大半径
BEARING_WINDOW_DEG = 60.0    # 程序端点必须在机场的"背离方向"±此角度内
MAX_FIX_JUMP_NM = 250.0      # 相邻程序点距离上限, 超过说明匹配到了别处的同名航路点
PROCEDURE_RADIUS_NM = 220.0  # SID/STAR 所有点必须在机场此半径内, 否则整段程序作废
PROCEDURE_END_NM = 12.0      # STAR 末点必须贴近跑道, 超过说明程序被 CIFP 截断
REVERSE_PENALTY = 0.25       # 逆行航路的代价放大比例(0.25 = 同样的边多花 25% 距离)

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


def _bearing(lat1, lon1, lat2, lon2):
    """起点到终点的初始方位角(度, 0=正北, 顺时针)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _angle_diff(a, b):
    """两个方位角之差, 归一化到 [0, 180]."""
    d = abs((a - b) % 360.0)
    return d if d <= 180.0 else 360.0 - d


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
    """解析 earth_nav.dat 取导航台坐标 -> {名称: (lat, lon)}, 给 CIFP 端点兜底用."""
    table = {}
    with open(NAV_PATH, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.split()
            # 行格式: 类型 纬度 经度 海拔 频率 范围 航向 名称 机场 区域 ...
            if len(parts) < 8 or not parts[0].isdigit():
                continue
            try:
                if parts[0] in ("4", "6", "12", "13", "14", "15", "16", "2", "3"):
                    table.setdefault(parts[7], (float(parts[1]), float(parts[2])))
                elif parts[0] == "5":  # VOR 行: 类型 纬度 经度 海拔 频率 范围 磁差 名称 ...
                    table.setdefault(parts[7], (float(parts[1]), float(parts[2])))
            except (ValueError, IndexError):
                continue
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


def _node_coord(name):
    """
    航路图节点坐标访问器: 返回 (lat, lon) 或 None.
    航路图节点名在全球基本唯一, 直接取第一条记录即可; 缺坐标时查导航台表.
    """
    coord = _fix_coord(name)                # 先查航路点表
    return coord if coord is not None else _get_navaids().get(name)


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
        # 同名点离参考点过远说明是别处的同名航路点, 视为解析失败
        best = min(entries, key=lambda e: _nm_distance(near[0], near[1], e[0], e[1]))
        if _nm_distance(near[0], near[1], best[0], best[1]) <= MAX_FIX_JUMP_NM:
            return (best[0], best[1])
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
    把扁平的程序行按 "程序名" 分组 -> {程序名: {"runways": set(), "rows": [...] }}
    rows 保持文件中的原有顺序 (CIFP 内部按序号排序, 顺序即航段顺序).
    """
    grouped = {}
    for cols in rows:
        if len(cols) <= _COL_PATH_TERM:
            continue
        name = cols[_COL_NAME].strip()                 # 程序名, 去掉尾部空格
        if not name:
            continue
        entry = grouped.setdefault(name, {"runways": set(), "rows": []})
        runway = cols[_COL_RUNWAY].strip()             # "RW18L" 或空串
        if runway:
            entry["runways"].add(runway.replace("RW", ""))  # 只留纯跑道号
        entry["rows"].append(cols)
    # CIFP 里同一程序名会重复出现多个块(跑道相关/过渡段), 按行首序号排序并去重,
    # 否则会拼接出 "起点-终点-起点-终点" 的折返航路.
    for entry in grouped.values():
        seen = set()
        unique = []
        for cols in entry["rows"]:
            key = (cols[0], tuple(c.strip() for c in cols[4:12]))  # 序号 + 航段主体
            if key in seen:
                continue
            seen.add(key)
            unique.append(cols)
        unique.sort(key=lambda c: int(re.sub(r"\D", "", c[0]) or 0))  # 按 "STAR:010" 的数字排序
        entry["rows"] = unique
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

        coord = _fix_coord(fix, airport, prev_coord) or navaids.get(fix)  # 先航路点表, 再导航台表
        if coord is None:
            alt_fix = cols[8].strip()                # 第 4 列无坐标时用第 8 列兜底
            if alt_fix and alt_fix != fix and not alt_fix.startswith("RW"):
                coord = _fix_coord(alt_fix, airport, prev_coord) or navaids.get(alt_fix)
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
    result = []
    for node in graph:
        coord = _node_coord(node)
        if coord is None:
            continue                                  # 没坐标的节点无法计算距离
        d = _nm_distance(lat, lon, coord[0], coord[1])
        if d <= limit_nm:
            result.append((d, node))
    result.sort()
    return result


def _dijkstra(graph, source, target, reverse_penalty=0.0, _unused=None):
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
    t_coord = _node_coord(target)

    def edge_cost(a, b):
        """a->b 的距离代价; 无坐标时用常数 100 海里保证搜索继续进行。"""
        ca, cb = _node_coord(a), _node_coord(b)
        if ca and cb:
            return _nm_distance(ca[0], ca[1], cb[0], cb[1])
        return 100.0

    def heuristic(n):
        """A* 启发式: 节点到目标的直线距离, 可采纳, 保证结果最优。"""
        nc = _node_coord(n)
        if nc and t_coord:
            return _nm_distance(nc[0], nc[1], t_coord[0], t_coord[1])
        return 0.0

    open_heap = [(heuristic(source), 0.0, source, 0)]   # (f, g, 节点, 逆行次数)
    best_g = {source: 0.0}
    prev = {source: None}
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
        for nbr, is_reverse in [(n, False) for n in graph.get(node, {})] + \
                               [(n, True) for n in _reverse_edges().get(node, ())]:
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
    for name, info in procedures.items():
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
        # 端点必须在航路图上, 否则以后再用就近点接入
        on_airway = bool(_get_airways().get(end_fix))  # 有出边才算真正接入航路网
        d_to_origin = _nm_distance(airport_coord[0], airport_coord[1],
                                   end_coord[0], end_coord[1])   # 端点离开本场的距离
        # 端点应朝"另一机场"方向延伸: 方位偏差越小越好
        want = _bearing(airport_coord[0], airport_coord[1], other_coord[0], other_coord[1])
        if is_sid:
            got = _bearing(airport_coord[0], airport_coord[1], end_coord[0], end_coord[1])
        else:
            got = _bearing(end_coord[0], end_coord[1], airport_coord[0], airport_coord[1])
        dev = _angle_diff(got, want)
        if dev > BEARING_WINDOW_DEG:
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
        # 完整性判据: 程序末端必须飞到机场附近, 否则是 CIFP 截断的残缺程序
        # (AND01A 只剩 AND->D202X 两段, 末点在机场 35nm 外, 不构成完整进近)
        # STAR 末点必须回到机场附近才算完整; SID 末点恰恰是远处的交接点, 不适用.
        end_dist = _nm_distance(airport_coord[0], airport_coord[1], pts[-1][1][0], pts[-1][1][1])
        if not is_sid and end_dist > PROCEDURE_END_NM:
            continue
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

    best = None   # (评分, 完整点列表, 元信息)

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

            # 端点在航路图上就直接用; 否则接最近的航路点(记额外代价)
            entry_penalty = 0.0
            entry = sid_end
            if not graph.get(sid_end):
                near = [(d, n) for d, n in _nearest_airway_nodes(*sid_pts[-1][1]) if graph.get(n)]
                if not near:
                    continue
                entry_penalty += near[0][0]
                entry = near[0][1]

            exit_penalty = 0.0
            exit_node = star_start
            if not graph.get(star_start):
                near = [(d, n) for d, n in _nearest_airway_nodes(*star_pts[0][1]) if graph.get(n)]
                if not near:
                    continue
                exit_penalty += near[0][0]
                exit_node = near[0][1]

            # 航路网络最短路径
            path, rev_count = _dijkstra(graph, entry, exit_node, REVERSE_PENALTY)
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
                total_nm = 0.0
                for i in range(len(full) - 1):
                    a = airports.get(full[i]) or _node_coord(full[i])
                    b = airports.get(full[i + 1]) or _node_coord(full[i + 1])
                    if a and b:
                        total_nm += _nm_distance(a[0], a[1], b[0], b[1])
                direct = _nm_distance(dep_coord[0], dep_coord[1], arr_coord[0], arr_coord[1])
                detour = total_nm / max(direct, 1.0) - 1.0  # 0 = 完全直线
            # 评分: 总航程最短者最优; 逆行航路加罚; 端点接入距离加罚.
            score = total_nm + 60.0 * rev_count + entry_penalty + exit_penalty

            if best is None or score < best[0]:
                best = (score, full, (sid_name, star_name, len(middle)))
        if tried >= max_pairs:
            break

    # --- 7. 兜底: 没有可用 SID/STAR 时, 直接机场 -> 最近航路点 -> 航路 -> 最近航路点 -> 机场 ---
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

    dep = sys.argv[1] if len(sys.argv) > 1 else "ZBAA"
    arr = sys.argv[2] if len(sys.argv) > 2 else "ZSPD"
    t0 = time.time()
    r = find_route(dep, arr)
    print(f"{dep} -> {arr}  ({len(r)} 个点, {time.time() - t0:.2f}s)")
    print(r)
    if len(r) >= 3:
        print("SID 终点:", r[1], "| STAR 起点:", r[-2])
    print("带航路名:", route_with_airways(r))
