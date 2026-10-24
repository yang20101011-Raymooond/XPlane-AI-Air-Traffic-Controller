# X-Plane AI Air Traffic Controller (AI-ATC)

在 X-Plane 12 里做 AI 空管：C++ 插件读取飞行状态 → Python 端做航路规划 + 语音识别 → LLM 生成 ATC 指令。

---


## 异地开发注意（2026-10-24）

原硬编码路径已全部去掉：

- **`read-IAS.cpp`** 写 JSON 的目录：编译期由 CMake 传入（= 仓库根），运行时可用环境变量 `AIATC_DIR` 覆盖。
- **`main.py`** 读写的 `flight_data.json` / `clearance.json`：默认脚本所在目录，同样认 `AIATC_DIR`。
  两边默认指向同一目录，正常情况下**什么都不用改**；只有 X-Plane 和仓库不在同一位置时才会用到这个变量。
- 导航数据（CIFP 等）：`XP_DATA_DIR`；插件部署：`XP_PLUGIN_DIR`。
- 要保证跨机能跑：**X-Plane 启动前** `export AIATC_DIR=...`（如果非默认），然后 `./bootstrap.sh` 重编一次。

换目录的完整做法：

```bash
export AIATC_DIR="$PWD"                              # 插件和 Python 共用数据目录（默认已是仓库根，可不设）
export XP_DATA_DIR="$HOME/X-Plane 12/Resources/default data"
cmake -S . -B build -DXP_PLUGIN_DIR="$HOME/X-Plane 12/Resources/plugins"
cmake --build build
```

## 异地开发：三步

> venv 不能跟着仓库走（里面是绝对路径 + 平台相关的二进制），所以只提交 `requirements.txt`，新机重装一次即可。
> 下面假设你已经 `git clone` 了本仓库。

### 1. 装依赖 + 编译（一条命令）

```bash
cd XPlane-AI-Air-Traffic-Controller
./bootstrap.sh                                   # X-Plane 默认在 ~/X-Plane 12
./bootstrap.sh "/Volumes/外接盘/X-Plane 12"       # 装别处时传路径
```

它会：建 `.venv` → `pip install -r requirements.txt` → cmake 配置 → 编译并部署 `mac.xpl`。

### 2. 设 API Key（不入库）

`main.py` 从环境变量读 DeepSeek key：

```bash
export LLM_API_KEY_AI_ATC_V1=你的key      # 想长期生效就写进 ~/.zshrc
```

### 3. 运行

```bash
source .venv/bin/activate
python main.py
```

第一次录音时 macOS 会弹麦克风授权，点允许。

---

## 手动做法（不用脚本时）

```bash
# Python
python3.12 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# CMake（需要 cmake >= 3.16；没装可以 pip install cmake）
cmake -S . -B build -DXP_PLUGIN_DIR="$HOME/X-Plane 12/Resources/plugins"
cmake --build build
```

日常只改 C++ 时：`cmake --build build`（自动编译 + 复制成 `mac.xpl` 到插件目录）。

## 仓库里有什么

| 路径 | 说明 |
|---|---|
| `SDK/` | 官方 X-Plane Plugin SDK 4.3.0（含 Mac framework），**已入库**，新机无需另下 |
| `CMakeLists.txt` | `xp_plugin(名字 部署目录 源文件...)` 一键生成插件目标 |
| `read-IAS.cpp` | 读指示空速的插件示例 |
| `main.py` | 主程序：录音 → Whisper 转写 → LLM 生成 ATC |
| `route_finder.py` | 基于 X-Plane 导航数据（earth_awy/fix/aptmeta + CIFP）的航路规划 |
| `getSIDandOthers.py` | 从 CIFP 里取 SID |
| `fetch_routes_fpd.py` | 从 FlightPlanDatabase 抓航路存进 `routes.db` |
| `requirements.txt` / `bootstrap.sh` | Python 依赖 / 新机一键初始化 |
| `.vscode/` | 构建任务与 IntelliSense 配置（已改成相对路径） |

## 路径与配置

- **X-Plane 安装目录**：Python 侧用环境变量 `XP_DATA_DIR`（默认 `~/X-Plane 12/Resources/default data`）；
  C++ 侧用 `-DXP_PLUGIN_DIR=...` 或环境变量 `XP_PLUGIN_DIR`（默认 `~/X-Plane 12/Resources/plugins`）。
- **不提交**的东西：`.venv/`、`build/`、运行日志 `flight_data.json` / `flight_debug.log` / `clearance.json`、`__pycache__/`。
- 大文件提醒：`torch` 约 1–2 GB，装依赖会慢一点；`openai-whisper` 首次运行还要下载模型权重。

## 注意事项

- 插件产物必须叫 `mac.xpl` 并放在 `Resources/plugins/<插件名>/`，X-Plane 不读源码目录。
- 改插件名后，旧的部署文件夹不会自动删，要手动清掉，否则 X-Plane 里会出现幽灵插件条目。
- Windows 上编译需要在 VS / MSVC 里做，`-bundle` 链接参数是 macOS 专用；本仓库当前只保证 macOS 可编译。
