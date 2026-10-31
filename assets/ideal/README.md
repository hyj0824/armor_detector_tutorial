# 理想正例

这 4 张图片来自课程数据集的 test split，目标较大、灯条近似平行。完成全部 TODO 后，默认参数应准确框到标注装甲板。

| 文件 | 颜色 | 原文件 | 用途 |
| --- | --- | --- | --- |
| `ideal_blue_near.jpg` | 蓝 | `ZJU-5154.jpg` | 近距离、大目标 |
| `ideal_blue_robot.jpg` | 蓝 | `1582760592.jpg` | 机器人侧面、暗背景 |
| `ideal_red_robot.jpg` | 红 | `09409.jpg` | 赛场机器人、中等距离 |
| `ideal_red_front.jpg` | 红 | `03052.jpg` | 正面、大目标 |

手动复现某一项：

```bash
./build/armor_detector assets/ideal/ideal_blue_near.jpg --type image --color blue
./build/armor_detector assets/ideal/ideal_red_front.jpg --type image --color red --no-gui
```

`ideal_samples` CTest 会检查检测框与人工标注框的 IoU（要求 ≥ 0.65），不要求灯条 mask 完全填满。灯条高亮中心缺失是 `REPORT.md` 的分析题，不是必做代码。
