#!/usr/bin/env bash
# ============================================================================
#  AI-ATC 新机初始化脚本（macOS / Linux）
#
#  用途：克隆仓库后，一条命令建好 Python 虚拟环境、装依赖、配置 CMake。
#  用法：
#      ./bootstrap.sh                          # 自动找 ~/X-Plane 12
#      ./bootstrap.sh "/Volumes/Ext/X-Plane 12"
#
#  它做四件事：
#    1. 检查 Python 3.12（没有就提示怎么装）
#    2. 建 .venv 并 pip install -r requirements.txt
#    3. 检查 cmake
#    4. cmake configure（把 X-Plane 插件目录传进去）+ 编译
#  重复运行是安全的（venv 已存在就复用，cmake 会重新配置）。
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")"

PY_MIN="3.12"
XP_ROOT="${1:-${XP_ROOT:-$HOME/X-Plane 12}}"
PLUGIN_DIR="$XP_ROOT/Resources/plugins"

echo "== 1/4 Python =="
if command -v python3.12 >/dev/null 2>&1; then
    PY=python3.12
elif command -v python3 >/dev/null 2>&1 && python3 -c "import sys; sys.exit(0 if sys.version_info[:2] >= (3, 12) else 1)"; then
    PY=python3
else
    echo "✗ 需要 Python $PY_MIN 或更高。macOS 上推荐：" >&2
    echo "    curl -LsSf https://astral.sh/uv/install.sh | sh && uv python install 3.12" >&2
    echo "  或 brew install python@3.12" >&2
    exit 1
fi
$PY --version

echo "== 2/4 虚拟环境 =="
if [ ! -d .venv ]; then
    "$PY" -m venv .venv
    echo "  已创建 .venv"
else
    echo "  .venv 已存在，复用"
fi
VENV_PY=".venv/bin/python"
[ -x "$VENV_PY" ] || { echo "✗ 找不到 $VENV_PY" >&2; exit 1; }
"$VENV_PY" -m pip install --upgrade pip >/dev/null
"$VENV_PY" -m pip install -r requirements.txt

echo "== 3/4 CMake =="
if ! command -v cmake >/dev/null 2>&1; then
    if [ -x "$("$VENV_PY" -c 'import cmake,os;print(os.path.join(os.path.dirname(cmake.__file__),"data","bin","cmake"))' 2>/dev/null || true)" ]; then
        echo "  使用 venv 里的 cmake"
        CMAKE="$("$VENV_PY" -c 'import cmake,os;print(os.path.join(os.path.dirname(cmake.__file__),"data","bin","cmake"))')"
    else
        echo "✗ 没有 cmake。推荐：$VENV_PY -m pip install cmake" >&2
        exit 1
    fi
else
    CMAKE=cmake
fi
"$CMAKE" --version | head -1

echo "== 4/4 CMake 配置 + 编译 =="
if [ ! -d "$PLUGIN_DIR" ]; then
    echo "  ⚠ 没找到 X-Plane 插件目录：$PLUGIN_DIR"
    echo "    （编译仍会成功，只是不会自动部署 mac.xpl；用 ./bootstrap.sh \"<X-Plane 12 路径>\" 重跑）"
fi
"$CMAKE" -S . -B build -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DXP_PLUGIN_DIR="$PLUGIN_DIR" -DAIATC_DIR="$(pwd)"
"$CMAKE" --build build

cat <<EOF

✔ 完成。日常用法：
    source .venv/bin/activate           # 进入虚拟环境
    export LLM_API_KEY_AI_ATC_V1=...    # main.py 需要这个环境变量（DeepSeek API key，不要写进文件）
    export AIATC_DIR="$(pwd)"           # 插件与 Python 共用的数据目录（不设则用编译时嵌入的本仓库路径）
    export XP_DATA_DIR="$XP_ROOT/Resources/default data"   # 可选，默认就是 ~/X-Plane 12/...
    python main.py

    cmake --build build                 # 改完 .cpp 重新编译 + 自动部署
EOF
