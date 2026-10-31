# armor_aim — 一个能跑通全链路的自瞄骨架（v0）

这不是一个"填空作业"，也不是一份参考实现。它是一台**能跑、能测、但明显不够好**的机器：

```text
仿真相机 ──▶ 颜色 mask ──▶ 灯条 ──▶ 装甲板候选 ──▶ 像素→角度 ──▶ 云台指令
                                                                  │
                                                      逐帧 trace  ◀─┘
```

你的工作不是"补完 TODO"，而是**让它变好，并说清楚为什么好**。

## 现在能做什么

| | 状态 |
|---|---|
| 静止靶 4 m 稳定命中 | ✅ 命中率 1.00，7.0 命中/秒 |
| 5.5 m 远靶 | ❌ 0 命中（抬升角写死） |
| 平移 / 自转靶 | ❌ 0 命中 |
| 7 m | ❌ 连候选都没有（检测的物理极限） |
| 随机静止靶下的云台阶跃响应 | ✅ 收敛 0.17~0.26 s，无超调 |
| 逐帧记录、离线回放、真值对齐 | ✅ |

## v0 故意留下的四个缺口

| 缺口 | 症状 | 位置 |
|---|---|---|
| 没有距离（`TargetEstimate::distance` 恒为 0） | 抬升角是一个写死的常数，只在 3.4~5.5 m 附近成立 | `pipeline/pipeline.cpp` |
| 没有时序关联（每帧独立选"离画面中心最近"） | 自转靶每秒换板 2~3 次，指令抖 | 同上 |
| 没有角速度 / 提前量 | 平移靶命中率 0.17 | `TargetEstimate::yaw_rate` |
| 没有生命周期管理 | 暂停 / 断流 / session 变更后仍按旧目标发指令 | `app/main.cpp` |

**这四条就是课程的四个阶段。** 每条都有一个场景在等着它，不需要看大纲。

---

## 1. 装依赖（这一步本身就是练习）

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev nlohmann-json3-dev
```

两个库，两个来源：

- **OpenCV 4**：图像处理。由 `find_package(OpenCV 4 REQUIRED)` 找。
- **nlohmann-json 3.10+**：解析仿真共享内存里的 JSON。由 `find_package(nlohmann_json)` 找。

验证一下有没有装上：

```bash
pkg-config --modversion opencv4          # 期望 4.x
ls /usr/include/nlohmann/json.hpp        # 有就是装好了
```

如果 CMake 报 `Could not find a package configuration file provided by "nlohmann_json"`，
说明第二个没装或者装到了非标准路径——**先解决依赖，别去改 CMakeLists**。

---

## 2. 构建与测试

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

> 讲师额外要跑参考实现（学生用不到）时加 `-DBUILD_REFERENCE=ON`，
> 会多出一个 `build/armor_aim_ref`，用来标定阈值。见 `docs/stages.md`。

两个测试，都只锁**行为**，不锁实现：

| 测试 | 锁什么 |
|---|---|
| `core` | 角度符号约定（像素向右 → yaw 为负）、绕回、目标选择、开火门控 |
| `ideal_samples` | 4 张实拍正例的检测框 IoU ≥ 0.65 |

把 `vision/angles.cpp` 里的符号改反，`core` 会立刻红——这是它存在的唯一理由。

---

## 3. 跑起来

### 终端一：启动仿真

先准备好 `rm_aim_sim`（本工程默认在 `../rm_aim_sim`）：

```bash
cd ../rm_aim_sim
./run.sh -- --remote                       # 默认场景：4 m 静止靶
./run.sh -- --remote --config=/绝对路径/armor_detector_tutorial/scenarios/spin.cfg
```

必须**图形模式**（`--headless` 没有像素输出）。`--remote` 打开"本机自瞄控制"，
否则仿真会丢弃你的指令。

### 终端二：跑自瞄

```bash
./build/armor_aim --color=blue --no-gui                  # 只瞄准，不开火
./build/armor_aim --color=blue --fire --no-gui           # 瞄准 + 开火
./build/armor_aim --color=blue --fire --trace=out/t.jsonl
```

结束时会打印两行：

```text
[self] frames=256 with_target=165 target_rate=0.645 mean_detect_ms=4.31 mean_aim_error_deg=0.11
[sim]  fired=56 resolved=56 hit=56 hit_rate=1.000 kills=0
```

- `[self]` 是你自己这条链路的表现；
- `[sim]` 是仿真给出的真实命中统计，**这个才算数**。

> `target_rate` 在静止靶上只有 0.65，不是 bug：**命中后灯条会熄灭 50 ms**。
> 仿真自带的这个行为，正好在练"目标短暂丢失"。

---

## 4. 七个场景

`scenarios/*.cfg` 直接喂给仿真。数字来自 `bench/run.sh <场景> 8`（1440×1080，
`max_health=10000` 以免靶被打死截断数据），主指标是**命中/秒**：

| 场景 | 条件 | 命中率 | 命中/s | 失败在哪一层 |
|---|---|---:|---:|---|
| `static` | 4 m 静止 | **1.00** | **7.00** | —— |
| `lowlight` | 4 m，0.15× 曝光 | 1.00 | 7.12 | 没崩——想想为什么 |
| `far` | 5.5 m 静止 | **0.00** | **0.00** | 抬升角写死，只在 4 m 附近对 |
| `translate` | 5.5 m，1.5 m/s 横移 | **0.00** | **0.00** | 没有提前量 |
| `spin` | 4 m，4 rad/s 自转 | **0.00** | **0.00** | 没有目标关联 |
| `combined` | 4.5 m，1.2 m/s + 2.5 rad/s | 0.14 | 0.38 | 两个都缺（**附加题**） |
| `toofar` | 7 m 静止 | —— | 0.00 | **检测**的物理极限，不是配对 |

这张表就是课程骨架：**每个 ❌ 都对应一个具体的能力**。

### 为什么主指标是"命中/秒"

命中率会被"很少开火但很准"骗过去。同一个基线，在软一点的场景里只打 6 发、
中 4 发，命中率 0.80，看上去比谁都好。**命中/秒同时惩罚"打不准"和"不敢打"。**

### 检测的工作区间（`bench/sweep.py` 实测）

| 距离 | 3.0 | 3.5 | 4.5 | 5.5 | 6.0 | 7.0 |
|---|---:|---:|---:|---:|---:|---:|
| 检出率 | 0.03 | 0.68 | 0.55 | **1.00** | 0.00 | 0.00 |

| 曝光 | 0.10× | 0.40× | 1.00× | 2.50× |
|---|---:|---:|---:|---:|
| 检出率 | 0.03 | 0.67 | 0.66 | 0.67 |

两头都是悬崖，不是缓坡：3.0 m 处过曝白洞比形态学核还宽，6 m 处灯条只有 2 px 宽。

### 测量噪声（谈"改进"之前先读这个）

动态场景的命中率重复 5 次，波动约 **±26%**。所以 `0.40 → 0.45` 不能当改进，
`0.40 → 0.65` 才够，下结论至少要重复 3 次。检测耗时只抖 4%，性能类结论可以单次测。

细节和复现命令见 `docs/stages.md`。

---

## 5. 随机静止靶：专门测"云台转得怎么样"

固定场景每次起点都一样，看不出云台的真实性能。这个工具每次随机靶位再跑一轮：

```bash
python3 bench/random_target.py --runs 6 --duration 8
```

它随机：距离 4.0~5.5 m、横向 ±2.2 m（偶尔超出水平视场）、车型（小/大装甲），
并让 0 号装甲正面朝向云台（保证量到的是纯云台响应，不掺目标选择的歧义）。

每个随机靶位都是一次**阶跃输入**，于是能量出四件事：

| 指标 | 含义 | 实测（6 轮） |
|---|---|---|
| `initial_deg` | 阶跃幅度 | 4.1 ~ 20.3° |
| `settle_s` | 从第一帧到误差进入 0.6° 并稳住 0.3 s | **0.17 ~ 0.26 s** |
| `overshoot_deg` | 收敛过程中反向偏了多少 | **0.00** |
| `steady_deg` | 最后 1 秒的平均偏差 | **0.01 ~ 0.02°** |
| `hit_rate` | 仿真统计 | **1.00** |

想加难度就打开 `--rotation`：装甲朝向也随机，这时"两块板争画面中心"的歧义会回来，
你能看到换板次数飙升、稳态误差从 0.01° 变成几度。**这就是"选择"和"控制"两件事的分界线。**

---

## 6. 工具

```bash
# 逐帧 trace（JSONL），配合画图
./build/armor_aim --trace=out/t.jsonl --fire --no-gui
python3 tools/plot_trace.py out/t.jsonl              # 需要 pip install matplotlib
python3 tools/plot_trace.py out/t.jsonl --save=/tmp/t.png

# 录一段，离线反复跑同一个场景
./build/armor_aim --record=out/rec1 --max-frames=200 --no-gui
./build/armor_aim --replay=out/rec1 --no-gui         # 不用开仿真
```

| 工具 | 干什么 |
|---|---|
| `bench/run.sh <场景> <秒>` | 起仿真 → 采真值 → 跑自瞄 → 对齐 → 出指标 |
| `bench/report.py` | 把 `out/` 下所有结果汇总成一张表（可直接贴报告） |
| `bench/noise.py <场景> --runs N` | 量**测量噪声**：不先知道这个，所有 A/B 都是自欺欺人 |
| `bench/sweep.py distance/exposure` | 扫参数，画出检测工作区间 |
| `bench/random_target.py` | 随机静止靶，测云台的阶跃响应 |
| `bench/sim_smoke.py` | 校验 `rm_aim_sim` 的接口契约有没有变 |

```bash
python3 bench/report.py
python3 bench/noise.py translate --runs 5
python3 bench/sweep.py distance --values 3,3.5,4.5,5.5,6,7
python3 bench/random_target.py --runs 6
python3 bench/sim_smoke.py
```

`bench/run.sh` 还会调用 `bench/truth.py`，把 trace 和真值按 `sim_time` 对齐，算出
**"指令方向"到真实装甲中心的角度误差**——这是 `[self]` 里的 `err` 给不了的，
因为 trace 里的 `err` 只是"指令减云台实际角"，说明不了瞄得对不对。

> `truth.py` 读的是真值。**它和 `bench/sample_truth.py` 都是"裁判"用的，不许出现在自瞄代码里**——
> 拿真值去瞄准，等于作弊，而且在真机上根本没有这个信号。
> `frame.json` 本来就没有 `truth` 字段（仿真侧已经剔掉），这是唯一的隔离手段。

---

## 7. 代码地图

| 文件 | 教什么 | 你大概会改它 |
|---|---|---|
| `core/types.h` | 一帧的输入长什么样；为什么姿态要跟着帧走 | 少 |
| `io/capture.h` | 采集层抽象（换视觉源不动上面的代码） | 少 |
| `io/sim_client.cpp` | 共享内存协议、原子发布、看门狗 | 别动 |
| `io/sim_capture.cpp` | 协议 → `Observation` 的适配 | 少 |
| `io/replay_capture.cpp` | 离线回放 | 少 |
| `vision/preprocess.cpp` | HSV、通道差、**先闭后开**（见注释） | 多 |
| `vision/light_bar.cpp` | 轮廓、`minAreaRect` 角度约定、几何过滤 | 多 |
| `vision/armor.cpp` | 两灯条配对与归一化几何量 | 中 |
| `vision/angles.cpp` | 像素 → 相对角，三个坐标系的符号 | 中 |
| `pipeline/pipeline.cpp` | 目标选择、抬升角、开火门控 | **主要战场** |
| `app/main.cpp` | 主循环：取帧→检测→估计→指令，50 Hz 下发 | 中 |
| `telemetry/trace.cpp` | 逐帧结构化日志 | 中 |

`pipeline` 是**纯函数**：`Observation → Detection → TargetEstimate → Command`，
不碰文件、不碰时钟、不碰仿真。所以每个阶段都能离线单测，也不用为了改检测器去开 Godot。

---

## 8. 四个阶段与参照点

`docs/stages.md` 里写了 v0 缺的四件事（距离 / 关联 / 提前量 / 生命周期），
每一件都包含：现象、为什么错、需要什么信息、改哪儿、**验收命令**、常见陷阱。

同一份文档里还有**参照点**：基线 v0 和参考实现各是多少命中/秒，
以及这些数字是怎么测出来的（噪声、重复次数、为什么用命中/秒）。
它们不是分数线，是"这件事能做到多好"的两个真实数据点。

## 9. 已知缺口一览

按场景逼出来的顺序，不是按难度排的（详细验收见 `docs/stages.md`）：

| # | 缺口 | 卡住的场景 | 需要补什么 | 需要的信息 |
|---|---|---|---|---|
| 1 | 抬升角是常数 | `far`、随机靶的远近变化 | 弹道补偿 | 距离 |
| 2 | 没距离 | 同上 | PnP 或灯条视高反推 | 灯条几何 + 内外参 |
| 3 | 没关联 | `spin` / `combined` | 换板惩罚 + 状态估计 | 历史帧 |
| 4 | 没盯速率 | `translate` | 提前量 = f(距离, 弹速, 延迟) | 距离 + 速率 |
| 5 | 没生命周期 | 暂停 / 断流 / session 变更 | generation 计数 + 超时 | 时间 |

注意顺序：**1 和 2 是同一件事**（没有距离就没法算抬升，也没法算提前量）。
先拿距离，后面三件事才有意义。

---

## 10. 踩坑记录（讲师实测）

| 现象 | 原因 | 处理 |
|---|---|---|
| 一根灯条都检不到 | 灯条过曝成纯白，`S`/`B-R` 掉到 0，mask 里只剩两条细边；**先开运算会把细边抹掉** | 改成先闭后开（已修，见 `preprocess.cpp` 注释） |
| 3.46 m 能检出、3.46 m 稍微转一下板就检不到 | 白洞宽度和形态学核尺寸是同一量级，处在临界点 | 工作区间 **约 3.4~5.5 m**，边界很脆。核改 5 能救近处，但 `ideal_blue_robot.jpg` 直接掉到 0 个候选（`ideal_samples` 会红） |
| 6 m 以上完全检不到 | 灯条只有 2 px 宽，低于 `min_area` | 这是**检测**的极限，不是配对的极限 |
| 跑着跑着没有目标了 | 靶被打死了，灯条全灭 | 场景里设 `max_health=10000` |
| 仿真报 `max_health must be in [-1, 10000]` | 上限就是 10000 | 别写 100000 |
| `--channel=xxx` 不生效 | 它**不是**命令行参数 | 改 `scenarios/*.cfg` 的 `[ipc] channel` |
| `--headless` 收不到图 | 无像素输出 | 用图形模式，或 Xvfb |
| 仿真说通道被占用 | 同一通道只允许一个客户端 | 关掉上一个客户端 |
| `pkill -f godot` 把自己也杀了 | `-f` 匹配整条命令行，包括你自己的 shell | 用 `pgrep -x godot` |

---

## 11. 提交什么

| 文件 | 内容 |
|---|---|
| `QA.md` | 16 个自检问题，每题挂证据（命令 + 输出 + `evidence/` 文件） |
| `REPORT.md` | 六节工程日志：基线 / 逐阶段 / 失败归因表 / 消融 / 证伪 / AI 说明 |
| `evidence/` | 上面两份文件引用到的所有原始输出 |
| 你的代码 | 改在 `pipeline/`、`vision/`、`app/`、`io/` 里 |

**这些不是作业，是你的工作记录。** 不打分。

它们存在的唯一理由是：三个月后你只会记得"好像改过什么东西"，
而 `REPORT.md` 第 3 节那张失败归因表，就是那时候唯一能找到线索的地方。
一年后在真机上遇到问题，能救你的也是这个记录习惯——AI 不知道你们的云台
有机械间隙，也不知道电控改了哪个字节。

讲师会做的事只有两件：抽几条 `evidence/` 重跑（看是不是真的），
然后**让你讲**一段你自己的 trace 曲线。讲不出来，回去重做；讲错了，当场讲清楚。

---

## 12. 每次实验记什么

不要写"我实现了 XXX"。写这四行：

```text
假设：把 X 从 A 改成 B，因为……，预期 M 会……
改动：具体改了哪个文件的哪一行
实测：before/after 的两次 bench 输出（贴数字，不贴印象）
结论：保留 / 回滚，以及和预期的差异怎么解释
```

**回滚也是有效结果。** "AI 说这样更快，实测更慢，我回滚了" 比"我实现了"信息量大得多。

---

## 目录

```text
app/         主循环与命令行
bench/       跑分脚本、真值采样、真值对齐、随机靶、扫参、噪声、契约检查
core/        跨模块的数据结构
io/          采集层（仿真 / 回放）与共享内存协议
pipeline/    检测 → 估计 → 指令
docs/        阶段说明与阈值（stages.md）
reference/   讲师用的参考实现（默认不编译）
scenarios/   七个仿真场景配置
telemetry/   逐帧 trace
tests/       回归测试
QA.md        作业：16 道题
REPORT.md    作业：实验报告模板
tools/       画图
vision/      检测器（mask / 灯条 / 配对 / 像素→角度）
assets/      实拍正例，给检测器单测用
```
