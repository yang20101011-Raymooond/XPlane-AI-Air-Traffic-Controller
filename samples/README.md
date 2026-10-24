# 运行时数据样本（samples/）

这两个 JSON 是插件的**输出**（`read-IAS.cpp` 每帧覆写 `flight_data.json`）和主程序的**输入/回写**
（`main.py` 读 `clearance.json`，放了行时回写航路点）。真实文件在仓库根目录，**不入库**，
所以这里各放了一份样本，换机器时照着看格式 / 调试用。

| 文件 | 真实位置 | 谁写 | 谁读 |
|---|---|---|---|
| `flight_data.example.json` | 仓库根 `flight_data.json` | `read-IAS.cpp`（X-Plane 每帧） | `main.py` |
| `clearance.example.json` | 仓库根 `clearance.json` | `main.py`（LLM 生成放行） | `main.py` |

## 怎么用

- 只想看格式：直接打开这两个文件。
- 想在没开 X-Plane 的情况下试 `main.py` 的解析逻辑：把样本复制成真实文件名即可

  ```bash
  cp samples/flight_data.example.json flight_data.json
  cp samples/clearance.example.json  clearance.json
  ```

  （`git status` 会看到它们，但已被 `.gitignore` 忽略，不会误提交。）

## 字段说明

`flight_data.example.json`
- `indicated-airspeed` 指示空速（kt）
- `heading` 航向（度）
- `altitude` 高度（ft）
- `above ground level` 离地高度（**米**，`main.py` 里乘 3.28 换成英尺）
- `vertical speed` 垂直速度（ft/min，负值=下降）
- `latitude` / `longitude` 经纬度
- `wind speed` / `wind heading` 风
- `QNH` 修正海压（**Pa**，102273.26 ≈ 1023 hPa）
- `aircraft-registration` 注册号（`main.py` 用它当呼号/识别）

`clearance.example.json`
- `callsign` 呼号（字母拼读）
- `runway` 起飞跑道
- `SID` 离场程序名
- `Waypoints in route` 空格分隔的航路，首个=起飞机场，末尾=降落机场
- `initial_climb_ft` / `cruise_altitude_ft` 初始上升高度 / 巡航高度（ft）
- `squawk` 应答机应答码、`QNH`（hPa 字符串）、`Destination` 目的机场

## 注意

这些是**运行时文件**，不要用 `git add -f` 把根目录的真实 JSON 提交上来——
否则每飞一次模拟器仓库就变脏一次（之前 GitHub 网页上传那版就是这么来的）。
要更新样本，只改 `samples/` 下的 `.example.json`。
