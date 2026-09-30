# AI-ATC — X-Plane 12 插件 (macOS)

第一个脚本 `example_plugin.cpp` + 官方 SDK 4.3.0 + CMake。目标是做成由 LLM
驱动的 AI 空管(ATC)，自动回复玩家、并在玩家不会执行指令时给出提示教程。

> 作者: Raymond (高一) · 从 Windows 迁到 Mac。Windows 上曾写过 `first_plugin`。

## 目录(精简)

```
AI-ATC/
├── example_plugin.cpp    ← 第一个/当前插件脚本(生命周期 + 日志)
├── CMakeLists.txt        ← 构建 & 自动部署(想加模块就把 .cpp 加进 SOURCES)
├── SDK/                  ← 官方 X-Plane SDK 4.3.0(XPSDK430, Laminar 提供)
└── .vscode/              ← C++(也是) + Python 开发配置
```

## 怎么用

**第一次 / 改 CMakeLists 后:**
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

**构建:**
```bash
cmake --build build
```
构建完会自动把 `mac.xpl` 复制到
`/Users/aic06/X-Plane 12/Resources/plugins/example_plugin/mac.xpl`。

在 VS Code 里可直接 `Cmd+Shift+B`(已配好任务)。

**验证加载:** 启动 X-Plane → 看 `Log.txt`, 应有
`[example_plugin] XPluginStart called (SDK 4.3.0).`

## 环境说明(Mac)

| 组件 | 状态 | 来源 |
|------|------|------|
| clang / clang++ / make | ✅ | Apple 自带 |
| CMake | ✅ | pip(`~/Library/Python/3.10/bin/cmake`) Homebrew 缺失时的替代 |
| SDK | ✅ 4.3.0 | 官方 Laminar SDK 下载页 |
| 链接 | runtime | X-Plane 12 自带 XPLM.framework |

> 官方 SDK 下载页: developer.x-plane.com/sdk/plugin-sdk-downloads/
> 现已支持 SD 430(12.4+), 宏需 XPLM200…XPLM430 全定义(见 CMakeLists)。

## 路线图(AI-ATC)
1. ✅ 插件生命周期 + 日志 + 可加载
2. ⬜ 用 `XPLMDataRef` 读玩家飞行状态(空速/航向/高度)
3. ⬜ 接 LLM 生成自然 ATC 指令
4. ⬜ 检测玩家偏差 → 新手提示/教程
