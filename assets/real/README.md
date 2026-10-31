# 真实装甲板调试素材

图片选自课程数据集（test split），仅用于本课程的传统视觉调试。默认参数应准确识别的正例放在 `../ideal/`；这里保留更接近实际比赛难度的观察素材：

| 文件 | 目标颜色 | 用途 | 对应原文件 |
| --- | --- | --- | --- |
| `basic_blue_front.jpg` | 蓝 | 基础：近距离、灯条清晰 | `00460.jpg` |
| `basic_red_front.jpg` | 红 | 基础：近距离、背景明亮 | `SJ-top-r3-light-mp4-8.jpg` |
| `standard_blue_multi.jpg` | 蓝 | 综合：多个候选与暗背景 | `010340.jpg` |
| `standard_blue_background.jpg` | 蓝 | 综合：复杂赛场背景 | `000317.jpg` |
| `challenge_red_dark.jpg` | 红 | 挑战：目标小、曝光低 | `00000003.jpg` |
| `challenge_red_tilt.jpg` | 红 | 挑战：相机倾斜、背景复杂 | `O_1119.jpg` |

建议按“基础 → 综合 → 挑战”的顺序观察，每次只改一个参数：

```bash
./build/armor_detector assets/real/basic_blue_front.jpg --type image --color blue
./build/armor_detector assets/real/standard_blue_multi.jpg --type image --color blue --no-gui
./build/armor_detector assets/real/challenge_red_tilt.jpg --type image --color red --no-gui
```

必做部分由 `ideal_samples` CTest 检查。对本目录的基础、综合、挑战素材，记录在哪个阶段失败以及原因。传统 HSV + 几何方法无法稳定覆盖整套真实数据是正常现象，不要为了通过一张图把阈值调到产生大量误检。

自动测试同时使用程序生成的合成图和 `../ideal/` 中固定的真实正例。本目录的复杂素材不设置“必须检出”的断言，避免为了某一张图过拟合阈值。
