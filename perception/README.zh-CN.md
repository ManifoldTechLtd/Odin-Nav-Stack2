# 端侧巡检感知（Florence-2）

[English](README.md) | **中文**

端侧视觉巡检与 VLM 问答模块（**Florence-2** / **SmolVLM** / **Moondream2**）维护在独立的开源仓库中，以便其 Python / PyTorch 推理环境与本 ROS 导航工作空间保持解耦：

- **独立仓库地址：** [https://github.com/ManifoldTechLtd/Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)

> **本模块为可选组件。** 不装它，ONS2 完全可以正常导航、规划与避障。
> 接入感知**不需要**重编导航栈、不需要改 launch、不需要动任何导航参数
> —— 它以两个旁挂进程的形式通过 HTTP（8730 端口）对接。

**先判断你需要吗：**

| 需要接入的场景 | 可以跳过的场景 |
|---|---|
| 巡检需输出按清单的**合格/不合格结论**（阀门开关、仪表读数、指示灯颜色、铭牌 OCR） | 机器人只需要**到达航点** |
| 希望在相机话题上看到**标注图**（检测框 + 结论文字） | 由人工看原始视频流判断 |
| 相机能推送 RGB（`sendrgb: 1` + `sendrgbundistort: 1`） | 平台没有 RGB 图像流 |

---

## 与 ONS2 导航栈的协作数据流

`Odin-perception` 通过两个进程以 HTTP 方式与本导航栈对接：

```text
odin_ros_driver ──► /odin1/image/undistorted
                            │
              ros1/ros2 capture node (:8730) ──HTTP──► perception-run (Florence-2)
                            ▲                                   │
                            └───────── POST /annotate ◄─────────┘
                            │
              /inspection/annotated/compressed ──► Foxglove / RViz / rosbridge
```

1. **本导航栈侧前置条件：** 在 Odin 驱动的 `control_command.yaml` 中开启去畸变彩色图输出（`sendrgb: 1` 且 `sendrgbundistort: 1`），并用以下命令确认话题正常发布：
   ```bash
   rostopic hz /odin1/image/undistorted        # ROS 1
   ros2 topic hz /odin1/image/undistorted      # ROS 2
   ```
2. **感知侧安装与使用：** 请参阅 [`Odin-perception` 仓库 README](https://github.com/ManifoldTechLtd/Odin-perception) 中的 *Integrating with Odin-Nav-Stack2* 章节。

---

## 如何拉取到 `perception/` 目录（避免产生 `perception/Odin-perception/` 双层嵌套）

如果你希望直接把 `Odin-perception` 的代码拉取到当前的 `perception/` 目录下（而不是在 `perception/` 里面再多套一层 `perception/Odin-perception/` 文件夹），请在**本仓库根目录（`Odin-Nav-Stack2/`）**执行以下任一方式：

### 方式一：直接克隆到 `perception` 目录（推荐）

```bash
# 在 Odin-Nav-Stack2 仓库根目录下执行：
rm -rf perception
git clone https://github.com/ManifoldTechLtd/Odin-perception.git perception
```

### 方式二：与本仓库平级克隆（独立目录）

```bash
cd ..
git clone https://github.com/ManifoldTechLtd/Odin-perception.git
```
