# AI-ATC 开发备忘录 (macOS · X-Plane 12)

> Raymond 的个人备忘:常用命令 + 踩过的坑。改源码/配置前先看这里。
> 配合 `~/AI-ATC/CMakeLists.txt` 使用。

## 0) 最重要:让 cmake 每次都能用(已修好)

cmake 是用 pip 装的,在 `~/Library/Python/3.10/bin/`,**不在终端默认 PATH**。
已把这行写进 `~/.zshrc`(开新终端自动生效):

```bash
export PATH="$HOME/Library/Python/3.10/bin:$PATH"
```

- 如果你在**旧的/已开的**终端里打 cmake 报 `command not found`,要么 `source ~/.zshrc`,要么重开一个终端窗口。
- 验证:`cmake --version` → 应显示 cmake version 4.4.3。

## 1) 日常开发(三步闭环)

```bash
cd ~/AI-ATC                 # 1. 进项目根目录(必须有 CMakeLists.txt)
cmake --build build         # 2. 编译 + 链接 + 自动部署到 X-Plane(一步全做)
```

构建完,产物自动落到:
```
~/X-Plane 12/Resources/plugins/<插件名>/mac.xpl
```
启动 X-Plane 看效果;日志看 `~/X-Plane 12/Log.txt`(搜索插件名)。

> 在 VS Code 里也能按 `Cmd+Shift+B` 触发同一件事(已配好一键任务)。

## 2) 加一个全新插件 xxx.cpp

1. 写 `xxx.cpp`(要有 `XPluginStart` 那 5 个回调)
2. 在 `~/AI-ATC/CMakeLists.txt` 最底部加一行:
   ```cmake
   xp_plugin(xxx xxx xxx.cpp)
   ```
   (`xp_plugin` 是通用函数,自动帮你编译+带宏+链接 SDK+部署成 mac.xpl)
3. 跑 `cmake --build build`
4. ✅ 产物 = `~/X-Plane 12/Resources/plugins/xxx/mac.xpl`

**多个插件也照做**:一个插件 = 文件里加一行 xp_plugin。

## 3) 改名 old → new ⚠️(两步,别漏第二步)

cmake **只会新增**,**不会自动删旧部署文件夹**。改名后旧文件夹会残留成"幽灵插件":

1. 改 CMakeLists:`xp_plugin(new new new.cpp)`(改源码文件名也一样)
2. 重跑构建后,**手动清掉旧的部署文件夹**(必做):
   ```bash
   rm -rf ~/X-Plane\ 12/Resources/plugins/old
   # 或更安全: mv 到废纸篓
   mv ~/X-Plane\ 12/Resources/plugins/old ~/.Trash/
   ```
   `old` 换成你旧插件名。不清的话 X-Plane 会多出一个残留条目并可能报错。

## 4) 改 CMakeLists 之后,记得重新执行一次 configure

改了 `CMakeLists.txt`(加/删插件、改目录路径、改宏)后,光 `cmake --build build` 不一定能正确拾取所有改变,最稳的是重新 configure:
```bash
cd ~/AI-ATC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build
```
(如果你只改了某个 `xxx.cpp` **代码本身**,不用重 configure,直接 `cmake --build build` 即可增量编译。)

## 5) 常见报错 → 原因 → 修法

| 报错/现象 | 原因 | 修法 |
|---|---|---|
| `zsh: command not found: cmake` | 终端 PATH 没加 cmake | `source ~/.zshrc` 或开新终端 |
| 改了名/新文件但“没反应” | 改了源码/CMakeLists 没重跑 build | `cmake --build build` |
| X-Plane 里多了个旧插件名条目 | 改名没删旧部署文件夹 | 见章节 3,手动 rm/mv 掉旧文件夹 |
| 编译红字 `undefined symbols` | (改多文件时)漏列某源文件 | 在对应 xp_plugin 那行加上该 .cpp |
| `token is not valid binary operator` / `Platform not defined` | 平台宏传错(CMake 把 -D 串粘成一个) | 见 CMakeLists 里用“;分号列表”写法;Mac 须 `-DAPL=1 -DIBM=0 -DLIN=0` |

## 6) 插件加载的目录铁律(X-Plane 自己定的)

```
~/X-Plane 12/Resources/plugins/
   └── <插件文件夹名>/
         └── mac.xpl      ← macOS 必须叫 mac.xpl, 且是 Mach-O bundle
```
- 每个插件**独占一个以插件名命名的子文件夹**,别把 mac.xpl 堆在 plugins/ 根部。
- X-Plane **不读你的源码**,只加载编译好的 mac.xpl;源码放 `~/AI-ATC` 没关系。

## 7) 想手动看 clang++ 在干嘛(理解原理用,日常不必)

完整人工编译+链接+部署 = cmake 替你做的三件事(日常用 `cmake --build build` 即可,手动容易敲错且不增量):
```bash
# ① 编译 .cpp→.o
c++ -c xxx.cpp -I SDK/CHeaders -I SDK/CHeaders/XPLM -I SDK/CHeaders/Widgets -I SDK/CHeaders/Wrappers \
    -DXPLM200=1 -DXPLM430=1 -DAPL=1 -DIBM=0 -DLIN=0 -std=gnu++17 -arch arm64 -fPIC -o xxx.o
# ② 链接 .o→xpl bundle
c++ -bundle xxx.o -F SDK/Libraries/Mac -framework XPLM -framework XPWidgets -arch arm64 -o xxx.xpl
# ③ 部署
mkdir -p ~/X-Plane\ 12/Resources/plugins/xxx
cp xxx.xpl ~/X-Plane\ 12/Resources/plugins/xxx/mac.xpl
```

---
_备忘录 v1 · Sep 3 2026_
