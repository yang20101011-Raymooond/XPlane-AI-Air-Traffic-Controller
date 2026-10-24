# -*- coding: utf-8 -*-
# ↑ 声明源文件是 UTF-8 编码（Python 3 默认就是 UTF-8，这行是兼容旧约定的保险，可留可删）。

# ═════════════════════════════════════════════════════════════
# import 区：导入「模块」。import 只是把工具领进来，本身不执行任何网络/文件操作。
# ═════════════════════════════════════════════════════════════
import json              # JSON 模块：json.loads(文本→对象) / json.dumps(对象→文本)
import os                # os.path.expanduser / 拼接脚本所在目录
import sqlite3           # SQLite 驱动，Python 标准库自带，不用 pip 安装
import sys               # 与解释器有关的东西：sys.argv 是命令行参数列表
import urllib.error      # urllib 家族的「错误类」：HTTPError 等
import urllib.parse      # urllib 家族的「URL 工具」：urlencode 把字典拼成查询串
import urllib.request    # urllib 家族的「发请求」：Request 对象 / urlopen 发出去
# 注意：urllib.request 是子模块，必须写全路径；只 import urllib 是拿不到 urlopen 的。

BASE = "https://api.flightplandatabase.com"   # API 根地址：协议 https + 域名，没有结尾斜杠
                                              # 后面拼路径时自己加 "/"，所以这里别带 "/"
# 数据库文件路径：SQLite 的「库」就是这个文件本身。
# 放在脚本同目录，仓库搬到哪台机器都能用。
DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "routes.db")

ORIGIN = "EDDF"   # 默认起飞机场 ICAO 四字码（法兰克福）
DEST   = "ZBAA"   # 默认降落机场 ICAO 四字码（北京首都）


# ═════════════════════════════════════════════════════════════
# 一、HTTP：发 GET 请求，拿回 JSON
# ═════════════════════════════════════════════════════════════
def http_get_json(path, params=None):
    # def 定义函数；path 是必填参数，params=None 表示「可不传，默认没有查询参数」
    # 一次 HTTP 请求 = 请求行(GET /路径?查询串 HTTP/1.1) + 请求头 + 正文(GET 没有正文)
    # 一次 HTTP 响应 = 状态行(HTTP/1.1 200 OK) + 响应头 + 正文(这里正文是 JSON 文本)
    # 本函数返回 (解析后的对象, 响应头字典)；失败返回 (None, None)。

    url = BASE + path                                  # 字符串拼接：根地址 + 路径，如 ".../search/plans"
    if params:                                         # 字典非空才拼查询串；空字典/None 都算假
        # urllib.parse.urlencode 把 {"a":1,"b":"x y"} 变成 "a=1&b=x+y"
        # ⚠️ 不能手拼 f"?a={x}&b={y}"：& = ? 空格都是 URL 保留字符，值里出现就会把参数切坏
        # urlencode 会自动做百分号编码（空格→+，&=→%26%3D），这是 HTTP 层最容易踩的坑之一
        url += "?" + urllib.parse.urlencode(params)    # "?" 之后才是查询串，只拼一次

    # Request 只是「组装好的请求单」，此刻还没有联网
    req = urllib.request.Request(                      # 第一个参数是 URL
        url,
        headers={                                      # headers 是「请求头」字典：额外的元信息
            # Accept：告诉服务器「我能吃 JSON」，不写的话可能回 XML，json.loads 直接报错
            "Accept": "application/json",
            # User-Agent：标明客户端身份；不少 API 会拒绝空 UA（当成爬虫）
            "User-Agent": "AI-ATC-route-collector/1.0",
        },
    )
    # 注意：Request 默认方法就是 GET；如果要 POST，得传 data=b"..."（有 data 就自动变 POST）

    try:                                               # try/except：联网会失败，必须兜住异常
        # urlopen 是真正「发出去」的那一步；它返回一个「响应对象」（类似打开的文件）
        # timeout=30：30 秒内没回应就抛异常，否则网络卡死会永久挂住
        # with ... as resp：离开缩进块自动调用 resp.close()，不用手动关
        with urllib.request.urlopen(req, timeout=30) as resp:
            # resp.read() 返回 bytes（原始字节流），HTTP 正文本质就是字节
            # .decode("utf-8") 把字节按 UTF-8 规则解成 str
            # errors="replace"：碰到非法字节替换成 "�"，而不是抛 UnicodeDecodeError
            body = resp.read().decode("utf-8", errors="replace")
            # resp.headers 是「响应头」：Content-Type、X-Limit-Used 这类信息都在里面
            # dict(...) 转成普通字典，方便后面用 .get() 取值
            headers = dict(resp.headers)

    except urllib.error.HTTPError as e:                # HTTPError：服务器回话了，但状态码非 2xx
        # e.code 是状态数字：400/401 参数或鉴权、404 路径错、429 超限、500 服务器side错
        # ⚠️ HTTPError 是 Exception 的子类，这个 except 必须写在通用 Exception 之前
        print(f"  ✗ HTTP {e.code}  {url}")             # f"..." 把变量插进字符串
        return None, None                              # 早返回：失败就结束函数

    except Exception as e:                             # 兜底：DNS 失败 / 连接超时 / TLS 证书错误等
        print(f"  ✗ 网络错误：{e}")
        return None, None

    try:                                               # JSON 解析也可能失败，单独兜
        # json.loads：JSON 文本 → Python 对象
        #   {"a":1} → dict；[1,2] → list；"s" → str；123 → int；true/false → True/False；null → None
        return json.loads(body), headers               # return 多个值 = return 一个元组
    except json.JSONDecodeError:                       # 典型的「返回了 HTML 错误页」场景
        print(f"  ✗ 返回的不是 JSON，前 200 字符：{body[:200]}")   # body[:200] 切片取前 200 个字符
        return None, None


def search_plans(origin, dest, limit=100):
    # limit=100 是默认参数，调用时不传就等于 100（FPD 文档里最大值也是 100）
    # 返回列表中每项字段：id / fromICAO / toICAO / distance / waypoints(个数!) / likes
    #                     / downloads / popularity / createdAt / cycle / tags / user
    # ⚠️ 参数名必须叫 fromICAO、toICAO，写成 from=/to= 服务器会 500
    # 发请求，解包出两个返回值；_ 是「我不用这个值」的占位符写法（下划线并没有特殊含义）
    data, headers = http_get_json("/search/plans", {
        "fromICAO": origin,          # 字典的键就是 URL 参数名，值自动被 urlencode 编码
        "toICAO": dest,
        "limit": limit,              # 一次最多 100 条
        "sort": "popularity",        # 服务器只支持这种排序；没有 sort=downloads，所以下载量只能本地再排
    })
    if headers:                      # headers 为 None 说明请求失败或解析失败，跳过
        # .get() 取不到键时返回 None，不会像 headers["x"] 那样抛 KeyError
        print(f"  额度 {headers.get('X-Limit-Used')}/{headers.get('X-Limit-Cap')}")
    return data or []                # data 为 None（失败）或空时，or 返回 []，保证调用方拿到列表


def get_plan_detail(plan_id):
    # f-string 直接把 plan_id 插进路径："/plan/12345"
    # 详情的 route.nodes 里才是真正的航点列表，每项含 type/ident/name/lat/lon/alt/via
    data, _ = http_get_json(f"/plan/{plan_id}")        # 第二个返回值不要，用 _ 接住
    return data                                      # 失败时 data 是 None，原样往上抛


# ═════════════════════════════════════════════════════════════
# 二、选计划：优先要有 AIRAC cycle 的，再按下载量排
# ═════════════════════════════════════════════════════════════
def pick_best_plan(plans):
    # 返回 (选中的计划 dict, 候选池是否带 cycle 的 bool)
    if not plans:                                    # 空列表 → 假值
        return None, False

    # 列表推导式：[表达式 for 元素 in 序列 if 条件]，从 plans 里过滤出新列表
    # (p.get("cycle") or {}).get("ident")：cycle 为 None 时 → {}，再取 ident 得 None，
    # 这样链式 .get 不会因为 None.get(...) 抛 AttributeError
    # 非空字符串为真，所以这个表达式「有 cycle 名」才为 True
    with_cycle = [p for p in plans if (p.get("cycle") or {}).get("ident")]

    # 三元表达式：有带 cycle 的就用它，否则整池降级
    pool = with_cycle if with_cycle else plans
    # max(序列, key=函数)：对每个元素算 key，返回 key 最大的那个元素
    # lambda p: (...) 是匿名函数，元组 (下载量, 点赞, id) 逐位比较，前面字段优先
    # p.get("downloads") or 0：字段缺失或为 None 时按 0 算，避免 None > int 报 TypeError
    best = max(
        pool,
        key=lambda p: (
            p.get("downloads") or 0,
            p.get("likes") or 0,
            p.get("id") or 0,
        ),
    )

    if not with_cycle:                               # 一条都没有 cycle → 提示可信度风险
        # 括号内字符串相邻会自动连接成一条；f"{len(plans)}" 把条数插进去
        print(f"  ⚠️ 这 {len(plans)} 条计划全都没有 cycle，"
              f"退回按下载量选（结果可能较旧）")

    return best, bool(with_cycle)                    # bool() 把非空列表转成 True，空列表转成 False


# ═════════════════════════════════════════════════════════════
# 三、数据整形：从详情里抽出航点名
# ═════════════════════════════════════════════════════════════
def extract_waypoints(detail):
    # 链式 .get 同样是防空值：detail 没有 "route" → 得 {} → .get("nodes") → None → or [] → []
    # ⚠️ 不能写 detail["route"]["nodes"]：缺键抛 KeyError，route 为 None 抛 AttributeError
    nodes = (detail.get("route") or {}).get("nodes") or []

    waypoints = []                                   # 先建空列表，后面 append 往里塞
    for n in nodes:                                  # 遍历每个节点（dict）
        ident = n.get("ident")                       # 航点名，如 "ESOBU"
        if ident:                                    # 过滤掉 None 和 ""（空字符串为假）
            waypoints.append(ident)                  # append 只是加到列表末尾，不返回新列表
    return waypoints


# ═════════════════════════════════════════════════════════════
# 四、SQLite：一个文件就是一个数据库，不需要单独的服务器进程
# ═════════════════════════════════════════════════════════════
def open_db():
    # connect 只是「打开文件」，文件不存在就顺手新建一个空库（这里的「库」= 这个 .db 文件）
    conn = sqlite3.connect(DB_PATH)                  # 返回 Connection 对象，后面都靠它执行 SQL

    # execute 执行一条 SQL；三引号字符串允许 SQL 跨行书写
    # CREATE TABLE IF NOT EXISTS：表已存在就什么都不做（不报错、不清数据）
    # 列类型只有 TEXT / INTEGER / REAL / BLOB / NULL；SQLite 是「弱类型」，类型更像建议
    # PRIMARY KEY (origin, dest, route)：三列组合做主键 = 这三列相同的行只允许存在一条
    conn.execute("""
        CREATE TABLE IF NOT EXISTS routes (
            origin      TEXT NOT NULL,               -- NOT NULL：这一列不许写 NULL
            dest        TEXT NOT NULL,               -- 行尾 -- 是 SQL 注释
            route       TEXT NOT NULL,
            waypoints   TEXT,                        -- TEXT 可空；这里放的是 JSON 字符串，不是数组
            plan_id     INTEGER,
            downloads   INTEGER,
            likes       INTEGER,
            cycle       TEXT,
            created_at  TEXT,
            source      TEXT DEFAULT 'flightplandatabase',   -- 不传该列时自动填这个值
            PRIMARY KEY (origin, dest, route)
        )
    """)
    # PRAGMA table_info(表)：查表的列信息，每行是 (cid, name, type, notnull, default, pk)
    # 集合推导式 {row[1] for ...}：只取第 2 列（列名），放进 set 方便 O(1) 判断
    # 遍历 execute 的结果即可，不用先 fetchall()
    existing = {row[1] for row in conn.execute("PRAGMA table_info(routes)")}
    for col, typ in [                                # 列表里是 (列名, 类型) 元组，可一次遍历两个变量
        ("downloads", "INTEGER"),
        ("likes", "INTEGER"),
        ("cycle", "TEXT"),
        ("created_at", "TEXT"),
    ]:
        if col not in existing:                      # 老库缺这列才补
            # f-string 拼 SQL 只用在「列名/类型」这种代码常量上；用户数据永远走 ? 占位符
            # ALTER TABLE ADD COLUMN 只能加列，不能改已有列的类型
            conn.execute(f"ALTER TABLE routes ADD COLUMN {col} {typ}")

    # ⚠️ 建表、改表这类「结构变更」也要 commit；不提交可能只留在内存/日志里
    conn.commit()
    return conn


def save_route(conn, origin, dest, route_str, waypoints,
               plan_id, downloads, likes, cycle, created_at):
    # 参数太多分两行写，这是合法的 Python 换行（括号内允许断行）
    # 返回本次实际新增的行数：0 = 主键冲突被 IGNORE，1 = 插入成功
    before = conn.total_changes                      # total_changes：该连接累计影响的行数（单调递增）
    conn.execute(                                    # execute(SQL, 参数元组)：两条语句分开写更清楚
        "INSERT OR IGNORE INTO routes "
        "(origin, dest, route, waypoints, plan_id, downloads, likes, cycle, created_at) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",        # 每条 ? 依次对应元组里的一项
        (
            origin,
            dest,
            route_str,
            # SQLite 没有 list 类型，所以序列化成 JSON 文本存；
            # ensure_ascii=False 保留中文原样，否则会写成 \uXXXX（也能存，只是不好读）
            json.dumps(waypoints, ensure_ascii=False),
            plan_id,
            downloads,
            likes,
            cycle,
            created_at,                              # 元组末尾这个逗号别删，否则就不是元组了
        ),
    )
    # ⚠️ ① 值永远走 ? 占位符：拼字符串时值里带单引号会语法错，
    #      更糟的是 "x'); DROP TABLE routes; --" 这种 SQL 注入
    # ⚠️ ② 插入后必须 commit，否则进程一退出更改就丢了（SQLite 默认自动开事务）
    conn.commit()
    return conn.total_changes - before               # 差值就是本次影响的行数


# ═════════════════════════════════════════════════════════════
# 五、主流程
# ═════════════════════════════════════════════════════════════
def main():
    # sys.argv 是列表：argv[0] 是脚本名，argv[1] 起才是命令行参数
    # 三元表达式 + len 判断：没传就用默认值，传了就用传进来的
    origin = sys.argv[1] if len(sys.argv) > 1 else ORIGIN
    dest   = sys.argv[2] if len(sys.argv) > 2 else DEST

    print(f"=== 搜索 {origin} -> {dest}（取 100 条，本地按下载量排序）===")
    plans = search_plans(origin, dest, limit=100)    # 关键字实参 limit=100，顺序无关可读性更好
    print(f"拿到 {len(plans)} 条计划")

    if not plans:                                    # 空列表 / None 都走这里
        print("没有计划可用，退出。")
        return                                       # return 不带值 = 结束函数（相当于提前退出）

    plan, has_cycle = pick_best_plan(plans)          # 元组解包：两个返回值分别赋给两个名字
    plan_id = plan["id"]                             # 这里用 [] 而不是 .get：id 一定存在

    print()
    print("下载量前 5（★ = 带 cycle）：")
    # sorted(..., key=..., reverse=True)：reverse=True 才降序；[:5] 切片取前五个（不足 5 个也不报错）
    for p in sorted(plans, key=lambda x: x.get("downloads") or 0, reverse=True)[:5]:
        cyc = (p.get("cycle") or {}).get("ident") or "-"   # 没有 cycle 名就显示 "-"
        mark = "★" if cyc != "-" else " "                  # 三元表达式挑前缀符号
        # {p['id']:<9} 是格式说明：[填充/对齐][宽度]；< 左对齐，9 是占 9 格
        # :<4、:<3 同理，让各列对齐；不写格式说明时默认就是 str() 的结果
        print(f"  {mark} id={p['id']:<9} dl={p.get('downloads', 0):<4} "
              f"likes={p.get('likes', 0):<3} wps={p.get('waypoints', 0):<4} "
              f"cycle={cyc}")

    cycle_ident = (plan.get("cycle") or {}).get("ident")   # 可能是 None
    created_at = plan.get("createdAt")               # API 字段是驼峰 createdAt

    print()
    print(f"选中 id={plan_id}（dl={plan.get('downloads', 0)}，"
          f"likes={plan.get('likes', 0)}，cycle={cycle_ident or '无'}）")
    if not has_cycle:                                # pick_best_plan 返回的 bool 用来提示
        print("  ⚠️ 这条计划没有 cycle 信息，导航数据版本未知")

    detail = get_plan_detail(plan_id)                # 第二次 HTTP 请求：取详情
    if not detail:                                   # None 或空 dict（失败）
        print("取详情失败，退出。")
        return

    waypoints = extract_waypoints(detail)
    if not waypoints:                                # 空列表
        print("这条计划没有航点，退出。")
        return

    route_str = " ".join(waypoints)                  # join：用空格把字符串列表粘成一条航路字符串
                                                     # （本版本只拼点名，不含 via 航路名）

    conn = open_db()                                 # 打开数据库连接
    added = save_route(                              # 写入并拿到新增行数
        conn, origin, dest, route_str, waypoints,
        plan_id,
        plan.get("downloads", 0),                    # 用 .get(键, 默认值)：键缺失也不报错
        plan.get("likes", 0),
        cycle_ident,
        created_at,
    )

    # execute(...) 返回游标 Cursor；fetchone() 取第一行，这里是一元组 (count,)，所以 [0]
    total = conn.execute("SELECT COUNT(*) FROM routes").fetchone()[0]

    print()
    print(f"航点（{len(waypoints)} 个）：")
    print(f"  {route_str}")
    print()
    print(f"本次新增 {added} 条，数据库共 {total} 条 → {DB_PATH}")

    # 读回来核对；字符串相邻即拼接，所以 SELECT 可以拆成两行写
    # WHERE 里同样用 ? 占位符，避免把用户输入拼进 SQL
    # ORDER BY 下载量降序 + LIMIT 1：只取最靠前那一行
    row = conn.execute(
        "SELECT origin, dest, route, plan_id, downloads, cycle, source "
        "FROM routes WHERE origin=? AND dest=? "
        "ORDER BY downloads DESC LIMIT 1",
        (origin, dest),                              # 可以是元组，也可以是列表，无 PII 时没区别
    ).fetchone()                                     # 没匹配到返回 None
    if row:                                          # 行是元组，按 SELECT 的列顺序用下标取
        print()
        print("库里读回（该 O-D 下载量最高的一条）：")
        print(f"  origin    = {row[0]}")             # 下标 0 起，和驼峰/下划线命名无关
        print(f"  dest      = {row[1]}")
        print(f"  route     = {row[2][:1024]}")       # 先取元组第 3 项字符串，再切前 1024 个字符，avoid overflow
        print(f"  plan_id   = {row[3]}")
        print(f"  downloads = {row[4]}")
        print(f"  cycle     = {row[5]}")
        print(f"  source    = {row[6]}")

    conn.close()                                     # 关闭连接，释放文件句柄（commit 之后的正常收尾）


# 只有「直接 python 跑这个文件」时 __name__ 才是 "__main__"；
# 被别的文件 import 时不会自动执行，方便复用里面的函数
if __name__ == "__main__":
    main()
