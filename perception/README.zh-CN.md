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

## 如何安装

### 方式 A：一条命令（推荐）

```bash
# 在 Odin-Nav-Stack2 仓库根目录下执行：
bash perception/setup.sh
```

脚本会把 `Odin-perception` 装到 `~/odin-perception`（**仓库之外**，不会动本目录的两份说明），
自动建隔离 venv、装钉死版本的依赖、下载 Florence-2 权重并跑自检。

常用参数：

| 参数 | 作用 |
|---|---|
| `--tag v0.1.0` | 钉版本（生产环境推荐） |
| `--dir <path>` | 装到指定目录 |
| `--cpu` | 强制 CPU 版 torch（无 GPU 平台） |
| `--check-only` | 只自检，不安装 |
| `--skip-weights` | 跳过 888MB 权重下载（离线设备） |

脚本是个**编排器**：实际调用 `Odin-perception` 自带的 `scripts/setup.sh`、
`scripts/download_models.sh` 与 `scripts/doctor.py`，安装逻辑不在本仓库重复一份。

### 方式 B：手动克隆到 `perception/` 目录（会覆盖本说明）

如果你更希望把源码放在本仓库内、又不想要 `perception/Odin-perception/` 双层嵌套：

```bash
# 在 Odin-Nav-Stack2 仓库根目录下执行：
rm -rf perception
git clone https://github.com/ManifoldTechLtd/Odin-perception.git perception
```

> ⚠️ 这条命令会**替换掉** `perception/` 目录，本目录的两份说明文件会被删除。
> 需要的话自己先备份。

### 方式 C：与本仓库平级克隆（独立目录）

```bash
cd ..
git clone https://github.com/ManifoldTechLtd/Odin-perception.git
```
