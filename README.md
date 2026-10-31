# armor_aim — 完整实现

```text
frame ──▶ mask ──▶ 灯条 ──▶ 装甲候选 ──▶ PnP ──▶ 跟踪 ──▶ 提前量+弹道 ──▶ 云台指令
                                  (vision/)  (solver) (tracker)  (pipeline)
```

同一道题的空骨架在 `main` 分支（仓库默认分支）。这份实现**不是标准答案**，
也有已知弱点（见文末）；怎么读它见 `main` 分支的 `docs/reading-guide.md`。

## 成绩

命中/秒，8 秒单次，1440×1080，`bench/run_all.sh`。三份实现的原始结果都在
`bench/frozen/`（含 `client-*.log`，可以原地复算）：

| 场景 | 骨架（`main`） | 本分支 | `reference/` |
|---|---:|---:|---:|
| `static` | 0.00 | **6.88** | 5.75 |
| `lowlight` | 0.00 | **6.50** | 5.50 |
| `far` | 0.00 | **9.00** | 2.50 |
| `translate` | 0.00 | **3.50** | 2.00 |
| `spin` | 0.00 | **3.62** | 2.12 |
| `combined` | 0.00 | **3.88** | 0.38 |
| `toofar` | 0.00 | 0.00 | 0.00 |

`far` 的 9.00 贴着武器射速上限（10 发/秒）。重复性（`bench/noise.py --runs 3`）：
`translate` 3.38/4.00/3.00、`spin` 2.88/3.38/3.12、`combined` 3.12/3.50/3.50。

`reference/` 是另一份独立实现（`-DBUILD_REFERENCE=ON` 才编译），只作第三个数
据点用；它在 `combined` 上的 0.38 是关联策略的差别，不是运动模型的差别。

## 1. 装依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev nlohmann-json3-dev
```

## 2. 构建与测试

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

## 3. 跑起来

```bash
# 终端一：起仿真（必须图形模式，--headless 没有像素输出）
cd ../rm_aim_sim
./run.sh -- --remote
./run.sh -- --remote --config=/绝对路径/armor_detector_tutorial/scenarios/spin.cfg

# 终端二：跑自瞄
./build/armor_aim --color=blue --fire --no-gui --duration=8
```

跑分：

```bash
bench/run.sh far 8                  # 单场景
TAG=before bench/run_all.sh         # 全部 7 个，结果进 out/before/
python3 bench/report.py             # 汇总成表
```

`bench/run.sh` 用独立通道 `armorbench`，启动前会清 `/dev/shm/rm_aim_sim_armorbench`，
所以不会和手起的仿真抢通道。

## 4. 场景

| 场景 | 条件 | 考的是 |
|---|---|---|
| `static` | 4 m 静止 | 闭环通了 |
| `lowlight` | 4 m，0.15× 曝光 | 检测在低曝光下没崩 |
| `far` | 5.5 m 静止 | S1 距离 + 弹道 |
| `translate` | 5.5 m，1.5 m/s 横移 | S3 提前量 |
| `spin` | 4 m，4 rad/s 自转 | S2 关联 + 速度估计 |
| `combined` | 4.5 m，1.2 m/s + 2.5 rad/s | 附加题 |
| `toofar` | 7 m 静止 | 检测的物理极限 |

任务书（现象 / 需要什么信息 / 改哪儿 / 验收 / 陷阱）和读码指南只在 `main` 分支上：
`docs/stages.md`、`docs/reading-guide.md`。这个分支不放副本，免得两处不同步。

## 5. 三个关键取舍

这三条都是实测踩出来的，每条的另一面都试过、都失败过：

| 取舍 | 另一面的后果 |
|---|---|
| 关联门限拆成"径向松 0.8 m / 切向紧 0.15 m" | 球形门限 0.25 m：PnP 深度噪声 0.1~0.3 m，一半的帧在重置跟踪 |
| 开火残差门限**按距离归一**（`0.03 × distance`） | 绝对值 0.06 m：`far` 只打出 2 发（跟踪误差其实只有 0.16°） |
| 位置**不**低通，直接进最小二乘 | 加 50 ms EMA：`spin` 从 3.6 掉到 0.0（自转靶切向在快速旋转，线性外推被低通坑死） |

另外两条容易被忽略的：

- **相机比云台旋转中心高 4 cm**。忽略它 5 m 处就有 0.46° 的系统性偏差，
  而纵向容差只有 0.68°。`solver` 里用 `camera_offset` 补掉了。
- **提前量里要有流水线延迟**。观测是过去的、指令是现在发的、弹丸是将来到的。
  只补发射延迟时 `translate` 的落点偏差是 5.4 cm，补上这一帧的间隔后是 4.0 cm。

## 6. 工具

```bash
./build/armor_aim --trace=out/t.jsonl --fire --no-gui   # 逐帧 trace（需 --trace）
python3 tools/plot_trace.py out/t.jsonl                 # 画图（需 matplotlib）
ARMOR_TRACE=1 bench/run.sh spin 8                       # 跑分时顺便录 trace

./build/armor_aim --record=out/rec1 --max-frames=200 --no-gui
./build/armor_aim --replay=out/rec1 --no-gui             # 离线复跑，不开仿真

python3 bench/noise.py translate --runs 5    # 测量噪声
python3 bench/sweep.py distance --values 3,4.5,5.5,6  # 检测工作区间
python3 bench/random_target.py --runs 6      # 随机静止靶：云台阶跃响应
python3 bench/sim_smoke.py                   # 校验仿真接口契约
```

### 评分口径

评分数据全部来自仿真发布的 `state.json`，**不读 trace**。`bench/run.sh` 按 80 Hz
把 state 落盘，`bench/truth.py` 出指标——所以换实现、删遥测、甚至一行都不打印的
程序都能被评分。

| 指标 | 怎么算 | 看什么 |
|---|---|---|
| 命中/s | 仿真统计的 `hit / 运行秒数` | **主指标**：同时惩罚打不准和不敢打 |
| 跟踪误差 | 逐 tick「云台实际指向 vs 理想弹道解」的中位数 | 解算+跟踪对不对（含伺服滑后） |
| 落点cm | 命中落点到装甲板中心的板面内距离 | 打得准不准（只算命中发） |
| miss_kind | `ground` / `body` / `expired` 各几发 | 打低了 / 打机身上 / 飞过头 |
| 帧率、仿真/墙钟 | 见 `bench/frozen/README.md` | 这一轮数据是否可信 |

两个坑：**不能拿"出膛方向 vs 装甲中心"当误差**（正确的弹道解故意瞄高 1.5~2.6°）；
`mean_aim_error_deg`（指令角减云台角）只说明伺服跟不跟得上，不是精度。

> `truth.py` / `sample_state.py` 读的是真值，只给评分用，不许出现在自瞄代码里。
> 注意 `state.json` 里带 `truth`，`io/sim_client.cpp` 把整份 state 都读进内存了——
> 这条隔离靠纪律，不靠接口。

## 7. 代码地图

| 文件 | 干什么 |
|---|---|
| `core/types.h` | 一帧的输入；姿态跟着帧走，弹道参数和相机偏移也在这里 |
| `io/sim_client.cpp` | 共享内存协议、原子发布、看门狗 |
| `io/sim_capture.cpp` | 协议 → `Observation` |
| `io/record.cpp` / `io/replay_capture.cpp` | 落盘 / 离线回放 |
| `vision/preprocess.cpp` | HSV + 通道差 + 先闭后开 |
| `vision/light_bar.cpp` | 轮廓 → 灯条，几何过滤 |
| `vision/armor.cpp` | 两灯条配对 + 归一化几何评分 |
| `vision/angles.cpp` | 像素 → 相对角（符号约定被 `core` 测试锁着） |
| `pipeline/solver.cpp` | 平面 PnP（两种间距 + 时间一致性）、云台/世界系、弹道解 |
| `pipeline/tracker.cpp` | 世界系关联 + 8 帧滑窗最小二乘（位置/速度/残差） |
| `pipeline/pipeline.cpp` | 提前量、角速度前馈、开火门控、观测过期 |
| `app/main.cpp` | 主循环：取帧→检测→估计→指令，50 Hz 下发 |
| `telemetry/trace.cpp` | 逐帧结构化日志 |
| `reference/` | 另一份独立实现，只作数据点 |

`detect()` 是纯函数；`estimate()` / `command()` 带状态（要历史帧才能估速度）。

## 8. 已知弱点

- **`translate` 的落点偏差 5.0 cm**，而 5.58 m 处容差只有 6.4~6.9 cm。余量很薄。
  速度估计是线性模型，目标掉头时有瞬态。
- **`combined` 的 3.88 靠的是关联策略**，不是整车运动模型。目标急停/急转时
  线性速度模型会退化，那时才需要估"整车旋转 + 平移"。
- **`hero` 大装甲没有专门验证**：代码里两种间距都试，但没有场景逼它。
- **`toofar` 还是 0**：7 m 处灯条只有 2 px 宽，卡在检测的物理极限。
- 没有线程、没有开火时机优化、PnP 的深度误差没有专门抑制。
