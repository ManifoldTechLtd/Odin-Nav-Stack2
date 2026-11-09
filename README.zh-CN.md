<h2 align="center">Odin Navigation Stack 2 (ONS2)</h2>

<p align="center">
  <img src="figures/logo.png" alt="Manifold Tech" height="50">
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="figures/astrall.png" alt="ASTRALL Dynamics" height="42">
</p>

<p align="center">
  <a href="README.md">English</a> | <strong>中文</strong>
</p>

<div align="center">
  <a href="https://manifoldtechltd.github.io/Odin-Nav-Stack2-Webpage/">
  <img src='https://img.shields.io/badge/Website-ONS2-blueviolet' alt='Website'></a>
  <a href="LICENSE">
  <img src='https://img.shields.io/badge/License-GPL--3.0-green' alt='GPL-3.0'></a>
  <a href="https://wiki.ros.org/noetic">
  <img src='https://img.shields.io/badge/ROS1-Noetic-orange' alt='noetic'></a>
  <a href="https://docs.ros.org/en/jazzy/">
  <img src='https://img.shields.io/badge/ROS2-Jazzy-blue' alt='jazzy'></a>
</div>

**Odin1** 是一款高性能空间感知模块，可提供**高精度 3D 建图**、**鲁棒的重定位**，以及包括 **RGB 图像**、**深度**、**IMU**、**里程计**和**稠密点云**在内的丰富感知数据流。ONS2 在 Odin1 的基础上，为地面平台提供自主**巡逻导航**栈，支持：

- **多航点自主巡逻**，具备动态避障能力
- **自然语言控制**与**视觉语言模型（VLM）**场景理解
- **双运行时**：同一套代码可运行在 **ROS1 Noetic** 和 **ROS2 Jazzy** 上

## 核心特性

- **高精度 SLAM 与持久重定位**（位于 Odin1 内部，未开源）  
  实时建图，并利用紧凑的二进制地图进行长期重定位。
- **动态障碍物感知导航**（完全开源）  
  带盲区障碍物记忆（`sliding_map`）的全局 A\* 规划，结合 NeuPAN 局部规划器，在杂乱环境中安全运动。
- **任务管理**（完全开源）  
  支持多航点任务，包括暂停、恢复、取消、返航、最终对准以及每个航点的停留。
- **自然语言大脑**（完全开源）  
  通过基于规则和基于 LLM 的规划，将文本命令路由到任务。可通过 `enable_motion` 或在运行时通过 `/brain/enable` 禁用运动。
- **视觉语言集成**（完全开源）  
  通过兼容 OpenAI 的 VLM 服务器进行场景问答。
- **双运行时，共享核心**  
  `ros_ws/`（ROS1 Noetic）与 `ros2_ws/`（ROS2 Jazzy）共享 NeuPAN 核心、话题/服务/动作名称以及配置档位。

## 新增内容

- **规划速度大幅提升**  
  即使在低算力平台上也能显著加快规划速度，运动执行更加平滑，避障更加敏捷。
- **升级版 VLM，支持接入自有模型**  
  可在你自己的工作站上连接更强大的视觉语言模型，实现更丰富的场景理解。
- **专为工业巡检打造**  
  针对真实工业巡检场景与长距离巡逻路线进行了大量专项优化。
- **端到端，完全开源**  
  完整的巡检工作流——从仿真到实机、从算法到监控平台。参见 [Real2Sim 教程](https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial)。
- **插件化架构**  
  整个导航栈围绕插件化设计构建，可轻松接入自定义规划器。
- **更多功能即将推出**  
  后续版本将持续带来更多新能力。

## 相关工作

- **[Odin-Nav-Stack](https://github.com/ManifoldTechLtd/Odin-Nav-Stack)** —— 本项目所基于的 ROS1 开源导航栈。
- **[SRU × Odin1: Mapless Navigation with Spatially-Enhanced Recurrent Memory](https://github.com/ManifoldTechLtd/SRU-Odin)** —— Odin1 上的无地图导航（SRU 后端）。
- **[NeuPAN（ManifoldTech 分支）](https://github.com/ManifoldTechLtd/NeuPAN)** —— ONS2 使用的局部规划器，带 ROS 封装。
- **[ManifoldTech Real2Sim 教程](https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial)** —— 使用 Q9000 将真实世界转换为 Isaac Sim 中的仿真环境。
- **[odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver)** 与 [Odin1 wiki](https://manifoldtechltd.github.io/wiki/Odin1/Cover.html) —— Odin1 驱动与文档。

# 支持的平台

| 机器人 | 运行时 | 状态 |
|---|---|---|
| Unitree Go2 | ROS1 Noetic | 支持（`scripts/ons2_go2.sh`，配置 `robots/go2/`） |
| ASTRALL SW01 | ROS1 Noetic / ROS2 Jazzy | 支持（`scripts/ons2_sw01.sh`，配置 `robots/sw01/`） |
| Dobot Lumo | ROS1 Noetic | 支持（`scripts/ons2_dobot_lumo.sh`，配置 `robots/dobot_lumo/`，默认 `dry_run=true`） |
| 你自己的机器人 | ROS1 或 ROS2 | 复制 `robots/_template/`；见 [03 · 接入你自己的机器人](docs/usage/03_integrate_your_robot.md) |

Unitree Go2 仅由 ROS1 栈支持。

# 局部规划器：NeuPAN 或 SRU

> [!NOTE]
> SRU 将在近期发布。敬请期待！

默认情况下，该栈在全局 A\* 路径上使用 **NeuPAN**（解析 DUNE + NRMP MPC）。ONS2 也可以用 **SRU** 端到端策略替代 NeuPAN，后者直接将深度图像和目标映射为速度指令。

| | NeuPAN（默认） | SRU（实验性） |
|---|---|---|
| 全局规划 | 是（A\*，`map_planner`） | 否 |
| 障碍物输入 | 来自 `sliding_map` 的 `/scan_360` | Odin1 深度图像 |
| 输出 | `/cmd_vel` | `/sru/cmd_vel`（5 Hz，机体坐标系），经运动门限传给 `/cmd_vel` |
| 模型 | 解析式：新底盘无需训练 | ONNX 策略：**必须针对你的机器人训练** |
| 选择方式 | 默认 | `--local-planner=sru` |

## 运行 NeuPAN

NeuPAN 是默认的局部规划器（`--local-planner=neupan`），适配新机器人底盘尺寸时无需任何神经网络训练：

- **Unitree Go2（ROS1）：**
  ``` shell
  bash scripts/ons2_go2.sh --map=NAME
  ```
- **ASTRALL SW01（ROS2 或 ROS1）：**
  ``` shell
  # ROS2（默认）
  bash scripts/ons2_sw01.sh --map=NAME

  # ROS1
  bash scripts/ons2_sw01.sh --ros1 --map=NAME
  ```
- **Dobot Lumo（ROS1）：**
  ``` shell
  bash scripts/ons2_dobot_lumo.sh --map=NAME
  ```

参数调优（速度、底盘尺寸、安全间隙、脱困策略）请参见 [02 · 参数调优](docs/usage/02_parameter_tuning.md)。

## 运行 SRU

> **SRU 需要训练。** 训练好的 ONNX 模型（策略和 VAE 编码器）**不包含**在本仓库中。请在 SRU 仓库中训练策略，然后将导出的模型部署到机器人上的 SRU 工作空间：
>
> **[ManifoldTechLtd/SRU-Odin](https://github.com/ManifoldTechLtd/SRU-Odin)** —— 训练（`Train/`）、部署（`Deployment/`）与移植指南。

在 SRU 模式下，该栈不规划全局路径。它将每个航点以 odom 坐标系发送给 SRU，并根据里程计距离判断是否到达（`nav_params.yaml` 中的 `sru_arrive_threshold_m`，默认 0.7 m）。路线分段、周期性重规划、恢复和热路径均被禁用。

在任一平台启动脚本中传入 `--local-planner=sru`，即可将局部规划器从 NeuPAN 切换为 SRU 策略：

- **Unitree Go2（ROS1）：**
  ``` shell
  export SRU_WS=/path/to/sru_ros1_workspace
  bash scripts/ons2_go2.sh --local-planner=sru --map=NAME
  # 可选：通过 --sru-policy=FILE 指定策略模型文件
  ```
- **ASTRALL SW01（ROS2 或 ROS1）：**
  ``` shell
  # ROS2（默认）
  export SRU_WS_ROS2=/path/to/sru_ros2_workspace
  bash scripts/ons2_sw01.sh --local-planner=sru --map=NAME

  # ROS1
  export SRU_WS=/path/to/sru_ros1_workspace
  bash scripts/ons2_sw01.sh --ros1 --local-planner=sru --map=NAME
  ```
- **Dobot Lumo（ROS1）：**
  ``` shell
  export SRU_WS=/path/to/sru_ros1_workspace
  bash scripts/ons2_dobot_lumo.sh --local-planner=sru --map=NAME
  ```

通用说明：
- 在 ROS2 下，启动脚本通过 `ros2_ws/scripts/sru/start_sru_ros2.sh` 与 `ros2_ws/scripts/sru/sru_cmd_vel_relay.py` 启动 SRU 策略节点与速度门控 relay（模型默认从 `$SRU_WS_ROS2/Deployment/models` 读取，可通过 `SRU_MODELS_DIR` 覆盖）。
- 在 ROS1 下，SRU 启动脚本默认使用 `sru_nav_go2_ros1`；可通过设置 `SRU_PKG` 和 `SRU_LAUNCH`（或在 `robots/<robot>/robot.yaml` 的 `planner.local` 中配置）指定对应机器人的 SRU 包和 launch 文件。
- `--navigation-mode` 在 SRU 模式下无效。
- SRU 输出机体坐标系下的 `Twist`，因此任何具备 `/cmd_vel` 桥接的机器人都可以使用。详见 [03 · 接入你自己的机器人](docs/usage/03_integrate_your_robot.md)。

# 安装与构建

两种运行时共用相同的克隆步骤和相同的 Odin1 USB 规则。然后按下面**其中一个**运行时章节操作。

## 通用步骤

### 1. 克隆仓库

``` shell
git clone --recursive https://github.com/ManifoldTechLtd/Odin-Nav-Stack2.git
cd Odin-Nav-Stack2
```

如果你没有使用 `--recursive` 克隆：

``` shell
git submodule update --init ros_ws/src/odin_ros_driver   # ROS1 Odin1 驱动（子模块）
git submodule update --init yolov5                       # 可选：目标检测
git submodule update --init odin_rtk_fusion              # 可选：RTK 输入
```

ROS2 的 Odin1 驱动 `ros2_ws/src/odin_ros_driver` 是 [manifoldsdk/odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver/) 的源码树内副本，不是子模块。

### 2. Odin1 USB 规则

``` shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```

添加下面这一行，然后重新加载规则并重新插入设备：

``` shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

``` shell
sudo udevadm control --reload && sudo udevadm trigger
```

## ROS1 Noetic（Ubuntu 20.04）

ROS1 运行 NeuPAN，要求 **Python ≥ 3.10**。Ubuntu 20.04 的系统 Python 为 3.8，因此需要将 ROS 和 NeuPAN 安装到 conda 环境中。

### 安装

``` shell
conda install -n base -c conda-forge mamba        # 然后重新登录 shell
mamba create -n neupan -y
mamba activate neupan
conda config --env --add channels conda-forge
conda config --env --remove channels defaults
conda config --env --add channels robostack-noetic
mamba install -n neupan ros-noetic-desktop colcon-common-extensions catkin_tools rosdep ros-dev-tools -y
pip install torch --index-url https://download.pytorch.org/whl/cpu   # 在 Jetson 上请使用 NVIDIA 的 wheel
pip install -e NeuPAN
python --version                                  # 必须为 3.10 或更高版本
```

告诉启动脚本哪个解释器运行 NeuPAN：

``` shell
export NEUPAN_PYTHON="$(which python)"
```

### 构建

先安装 Unitree SDK：`ros_ws/src/unitree_control` 会调用 `find_package(unitree_sdk2)`。然后构建所有 ROS1 包：

``` shell
bash ros_ws/src/odin_ros_driver/script/build_ros.sh
```

这等同于 `cd ros_ws && catkin_make -DCMAKE_BUILD_TYPE=Release`。

- **没有 Unitree SDK？** 只构建你需要的包：`catkin_make -DCATKIN_WHITELIST_PACKAGES="nav_interfaces;odin_ros_driver;map_planner;sliding_map;neupan_ros;mission_manager;atomic_move;sw01_ros_control"`。
- **Dobot Lumo** 为可选项。没有 Lumo SDK 时，`dobot_ros_control` 只构建原始 RPC 后端。
- 构建产物与机器架构绑定。更换主板后需要重新构建。

### 启动

``` shell
bash scripts/ons2_go2.sh --map=NAME                            # Unitree Go2 (ROS1)
bash scripts/ons2_dobot_lumo.sh --map=NAME                     # Dobot Lumo (ROS1)
```

停止：`bash scripts/ons2_go2_stop.sh`（Go2）或 `bash scripts/ons2_dobot_lumo_stop.sh`（Dobot Lumo）。

## ROS2 Jazzy（Ubuntu 24.04）

ROS2 面向 **ASTRALL SW01** 平台。它使用系统 Python 3.12，不需要 conda。

### 安装

``` shell
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-rosbridge-suite ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-std-srvs ros-jazzy-message-filters \
  ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime \
  ros-jazzy-nav2-map-server ros-jazzy-foxglove-bridge

python3 -m pip install --break-system-packages "numpy<2" scipy cvxpy cvxpylayers ecos \
  matplotlib rich pyyaml scikit-learn loguru "gctl==1.2" \
  torch --index-url https://download.pytorch.org/whl/cpu --extra-index-url https://pypi.org/simple
```

> 请保持 `numpy` 低于 2.0。NeuPAN/cvxpy 和 ROS2 的 `cv_bridge` 构建依赖于此。NeuPAN 不通过 pip 安装；启动脚本会将 `NeuPAN/` 添加到 `PYTHONPATH`。

### 构建与启动

``` shell
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
cd ..
bash scripts/ons2_sw01.sh --map=NAME                           # ASTRALL SW01 (ROS2)
```

停止：`bash scripts/ons2_sw01_stop.sh`。在手动打开的每个终端中使用 `ROS_DOMAIN_ID=98`；启动脚本默认会设置它。

## ROS1 与 ROS2 的差异

| 项目 | ROS1（Noetic） | ROS2（Jazzy） |
|---|---|---|
| 操作系统 | Ubuntu 20.04 | Ubuntu 24.04 |
| NeuPAN 使用的 Python | 系统 `python3` 或 `NEUPAN_PYTHON`（Python ≥ 3.10） | 系统 Python 3.12（`pip --break-system-packages`） |
| 构建 | `catkin_make` / `build_ros.sh` | `colcon build --symlink-install` |
| 发现机制 | `roscore`（由启动脚本启动） | DDS，`ROS_DOMAIN_ID=98`，无 `roscore` |
| 启动脚本 | `scripts/ons2_go2.sh`、`scripts/ons2_dobot_lumo.sh` | `scripts/ons2_sw01.sh` |
| `--navigation-mode` | 可选（默认 `replan_periodic`） | 可选（默认 `replan_periodic`） |
| 停止 | `scripts/ons2_go2_stop.sh`、`scripts/ons2_dobot_lumo_stop.sh` | `scripts/ons2_sw01_stop.sh` |
| tmux 会话 | `ons2_go2`、`ons2_dobot_lumo` | `ons2_sw01` |
| 任务 CLI / 交互控制台 | `ros_ws/scripts/test_nav_control_rospy.py` 或 `scripts/tools/mission_cli.sh` | `ros2_ws/scripts/test_nav_control_ros2.py` 或 `ros2 service call` |
| 机器人 | Go2、Dobot Lumo、SW01 | SW01 |
| Odin 驱动 | 子模块 `ros_ws/src/odin_ros_driver` | 源码树内副本 `ros2_ws/src/odin_ros_driver` |

# 文档

三份指南覆盖主要任务：

| # | 指南 | 用途 |
|---|---|---|
| 1 | [现有机器人快速开始](docs/usage/01_quick_start.md) | 在 Go2、SW01 或 Dobot Lumo 上启动该栈，并发送任务 |
| 2 | [参数调优](docs/usage/02_parameter_tuning.md) | 为各平台调整速度、底盘尺寸、间隙和跟踪参数 |
| 3 | [接入你自己的机器人](docs/usage/03_integrate_your_robot.md) | 接入新机器人：接口契约、底盘桥接、TF、启动 |

其他参考：

- 调优指南（详细）：[docs/TUNING_GUIDE.md](docs/TUNING_GUIDE.md)
- 系统架构：[docs/architecture/ONS2_ARCHITECTURE.md](docs/architecture/ONS2_ARCHITECTURE.md)
- 模块数据流与诊断：[docs/architecture/MODULE_DATAFLOW.md](docs/architecture/MODULE_DATAFLOW.md)
- Q9000 扫描建图与 PCD → PGM 转换教程：[docs/Q9000-tutorial/Q9000-tutorial.md](docs/Q9000-tutorial/Q9000-tutorial.md)（`scripts/map_processing/pcd_to_pgm.py`）
- 机器人档案规范：[docs/architecture/ROBOT_PROFILE.md](docs/architecture/ROBOT_PROFILE.md)
- 巡检感知（Florence-2）及其与导航栈的集成指南：[Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)
- Docker（Jetson，ROS1）：[docker/docker-compose.yml](docker/docker-compose.yml)

## 自然语言控制（odin_brain）

`odin_brain` 接收 `/brain/command` 上的文本并将其转换为任务。在 ROS2 中：

``` shell
mkdir -p ~/.config/odin_brain
cp ros2_ws/src/odin_brain/config/llm.env.example ~/.config/odin_brain/llm.env
chmod 600 ~/.config/odin_brain/llm.env        # 填入你的 LLM API key
ros2 run odin_brain check_llm                 # 验证 LLM 连接
ros2 run odin_brain brain_node                # 启动常驻节点
```

该大脑节点从不直接发布 `/cmd_vel`。它只调用任务服务。

## VLM 场景理解

启动 VLM 服务器（llama.cpp，兼容 OpenAI API），然后启动终端客户端：

``` shell
llama-server -m SmolVLM-500M-Instruct-Q8_0.gguf --mmproj mmproj-SmolVLM-500M-Instruct-Q8_0.gguf
ros2 launch odin_vlm_terminal odin_vlm_terminal.launch.py     # ROS2
roslaunch odin_vlm_terminal odin_vlm_terminal.launch          # ROS1
```

在资源有限的车载计算机上，请在配套机器上运行 VLM 服务器。

## 巡检感知（Florence-2）

> **可选模块 —— 不装它 ONS2 也能完整运行。** 接入感知是“两进程旁挂”，不是重新构建：
> 无需重编本栈、无需改 launch、无需动任何导航参数，随时装也随时拆。

**你需要它吗？**

| 需要 Odin-perception 的场景 | **不需要**的场景 |
|---|---|
| 巡检必须输出**合格/不合格结论**（阀门开关、仪表读数、指示灯颜色、铭牌 OCR） | 只需要机器人在航点间**导航** |
| 希望在相机画面上看到**标注图**（检测框 + 结论文字） | 靠人看视频流自己判断 |
| 你的 Odin1 相机能推 RGB（`sendrgb: 1`） | 平台没有 RGB 图像流 |

针对**结构化巡检任务**（如阀门是否关闭、仪表读数、指示灯颜色识别），巡检感知模块位于**独立的开源仓库**中，因此模型环境无需合并到本 ROS 工作空间内：

**➡️ [Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)**

它接收图像并对照 YAML/JSON 巡检清单返回可判定的结论，同时可发布带检测框和结论文字的标注图像。它通过两个进程以 HTTP 方式与本导航栈对接：

```
odin driver ──► /odin1/image/undistorted
                      │
        ros1/ros2 capture node (:8730) ──HTTP──► perception-run (Florence-2)
                      ▲                                   │
                      └───────── POST /annotate ◄─────────┘
                      │
        /inspection/annotated/compressed ──► Foxglove / RViz / rosbridge
```

**安装与集成步骤详见该仓库的 README** —— 请参考其中的 *Integrating with Odin-Nav-Stack2* 章节了解详细步骤、话题契约以及 ROS 1 / Jetson 注意事项。

本导航栈侧的前置条件：Odin 驱动需开启图像发布（即在驱动 `control_command.yaml` 中设置 `sendrgb: 1` 和 `sendrgbundistort: 1`），接入感知前请先用 `rostopic hz /odin1/image/undistorted` 验证。

> 上文的 `odin_vlm_terminal` 是另一条轻量级路径：它将图像转发给兼容 OpenAI 的 VLM 服务器进行自由问答。当你需要可重复、由清单驱动的巡检判定结果时，请使用 `Odin-perception`。

# 架构

该栈采用分层结构：Odin1 感知与 TF → `pointcloud_to_laserscan` → `sliding_map` 障碍物记忆 → A\* 全局规划（`map_planner`）→ NeuPAN 局部控制（`neupan_ros`）→ 底盘桥接，由 `mission_manager` 协调。同一时刻只允许一个节点发布 `/cmd_vel`。控制权通过 `/neupan/stop` 从 NeuPAN 移交给 `atomic_move`。

# 仓库结构 (Repository Structure)

```text
Odin-Nav-Stack2/
├── robots/                        # 单一机器人配置真源
│   ├── _template/                 # 新机器人接入模板 (robot.yaml + neupan/{config,planner}.yaml)
│   ├── go2/                       # Unitree Go2 档案与 NeuPAN 配置 (ROS 1)
│   ├── sw01/                      # ASTRALL SW01 档案与 NeuPAN 配置 (ROS 2)
│   └── dobot_lumo/                # Dobot Lumo 档案与 NeuPAN 配置 (ROS 1)
├── scripts/                       # 统一启动入口与运维工具
│   ├── ons2_go2.sh / ons2_go2_stop.sh
│   ├── ons2_sw01.sh / ons2_sw01_stop.sh
│   ├── ons2_dobot_lumo.sh / ons2_dobot_lumo_stop.sh
│   └── lib/ | brain/ | tools/ | systemd/ | map_processing/
├── ros_ws/                        # ROS 1 Noetic 工作空间
│   ├── scripts/                   # ROS 1 运行时脚本 (test_nav_control_rospy.py, waypoint_goal_bridge.py, sru/, ...)
│   └── src/
│       ├── ons2_bringup/          # 基于 robot.yaml 的 ROS 1 启动包 (go2, dobot, map_planner, neupan)
│       ├── nav_interfaces/        # 共享消息、服务与动作定义
│       ├── sliding_map/           # 360° 虚拟激光与盲区障碍物记忆
│       ├── map_planner/           # 全局 2D A* 规划器与激光时序滤波
│       ├── neupan_ros/            # NeuPAN 的 ROS 1 Action Server 封装
│       ├── mission_manager/       # 多航点任务状态机
│       ├── atomic_move/           # 到点闭环精对齐服务 (move / turn / align)
│       ├── unitree_control/       # Unitree Go2 SDK2 底盘速度桥
│       ├── dobot_ros_control/     # Dobot Lumo RPC 底盘速度桥与 O1L 启动配置
│       ├── odin_brain/            # 自然语言规划器与 Hermes Web 插件
│       └── odin_vlm_terminal/     # VLM 场景问答终端节点
├── ros2_ws/                       # ROS 2 Jazzy 工作空间 (与 ros_ws 核心包一一对应)
│   ├── scripts/                   # ROS 2 运行时脚本 (test_nav_control_ros2.py, waypoint_goal_bridge.py, sru/, ...)
│   └── src/
│       ├── ons2_bringup/          # 通用 profile 驱动入口 bringup.launch.py
│       ├── nav_interfaces/        # ROS 2 IDL 消息、服务与动作定义
│       ├── sliding_map/           # ROS 2 360° 虚拟激光与盲区障碍物记忆
│       ├── map_planner/           # ROS 2 全局 A* 规划器与点云/激光滤波
│       ├── neupan_ros/            # ROS 2 Action Server + ros2_shim.py 兼容层
│       ├── mission_manager/       # ROS 2 任务状态机与路径安全校验
│       ├── atomic_move/           # ROS 2 闭环对齐节点
│       ├── sw01_ros_control/      # ASTRALL SW01 SDK 底盘速度桥
│       ├── odin_brain/            # ROS 2 自然语言规划器
│       └── odin_vlm_terminal/     # ROS 2 VLM 场景问答节点
├── NeuPAN/                        # 共享 MPC + 解析式 DUNE 局部规划器核心库 (Python)
├── perception/                    # ManifoldTechLtd/Odin-perception 巡检感知仓库说明与拉取指南
├── docker/                        # Jetson (ROS 1) Docker 部署配置
└── docs/                          # 文档体系
    ├── usage/                     # 01_quick_start, 02_parameter_tuning, 03_integrate_your_robot
    ├── architecture/              # ROBOT_PROFILE.md, ONS2_ARCHITECTURE.md, MODULE_DATAFLOW.md
    └── TUNING_GUIDE.md            # 完整参数字典与症状排障手册
```

# 常见问题

## 如何检查重定位？

打开 RViz（ROS1 用 `rviz`，ROS2 用 `rviz2`），将固定坐标系设为 `map`，并添加 `TF` 显示。当 `map → odom → odin1_base_link` 连接成功时，表示重定位成功。

## 发布目标后，起点或目标点被占用

在 RViz 中检查膨胀地图（`/inflated_map`）。起点和目标点必须位于膨胀区域之外。调整 `robots/<robot>/robot.yaml` 中的 `navigation.inflation_radius`。

## 机器人接近目标时不停车

这是 NeuPAN 的容差问题。请参见 [02 · 参数调优](docs/usage/02_parameter_tuning.md) 和[调优指南](docs/TUNING_GUIDE.md)。

## ROS2 节点已运行但彼此不可见

所有节点必须使用相同的 DDS 域 `ROS_DOMAIN_ID=98`。在手动打开的每个终端中都设置它。

# 致谢

感谢 [ROS Navigation](https://github.com/ros-planning/navigation)、[NeuPAN](https://github.com/hanruihua/NeuPAN) 和 [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) 的优秀工作。

特别感谢 [hanruihua](https://github.com/hanruihua) 和 Manifold Tech 的 Odin1 团队。

# 许可证

ONS2 基于 [GNU General Public License v3.0](LICENSE) 发布。第三方组件保留各自的许可证；完整清单包括 NeuPAN（GPL-3.0）、Odin1 驱动（Apache-2.0）、`yolov5`（AGPL-3.0，可选子模块）以及 ASTRALL SDK（专有，不随本仓库分发）。再分发前请检查各组件的许可证。
