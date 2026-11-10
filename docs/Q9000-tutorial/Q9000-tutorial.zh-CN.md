# Q9000 建图扫描与处理指南

[English](Q9000-tutorial.md) | **中文**

> [!NOTE]
> 本指南虽然用 **Unitree Go2** 作为文件传输与安装高度参数的具体示例，但 Q9000 扫描、地图导出、
> PCD 转 PGM 栅格这套流程是**完全平台无关**的。任何运行 ONS2 的机器人（如 ASTRALL SW01、
> Dobot Lumo，或自研移动平台）都同样适用，与具体机型无关。

本文完整走一遍流程：用 **Q9000** 扫描环境 → 在 **MindCloud Studio** 里导出点云与重定位地图 →
转换成可用于自主导航的 2D 占据栅格地图。

---

## 1. 把 Q9000 连到工作站

用 Type-C 转 USB-A（或 Type-C 转 Type-C）数据线，连接 [Q9000](https://www.manifoldtech.cn/products/q9000) 头部背面的 USB-C 口与你的电脑。识别之后，电脑上会出现一个可移动磁盘，里面就是录制的工程文件夹。

![Q9000 底部接口](https://i.ibb.co/prvWBS9C/09.png)

> [!TIP]
> 建议用带独立显卡的工作站或笔记本。没有独显的超极本处理点云会慢很多。

---

## 2. 在 MindCloud Studio 中打开工程

在电脑上启动 **MindCloud Studio**，按下面步骤操作。（MindCloud Studio 的安装包与授权请联系留形科技支持获取。）

1. 左上角菜单栏点 **文件 → 新建工程 → 单点云工程**（`File → New Project → Single Point Cloud Project`）：

   ![MindCloud Studio 文件 -> 新建单点云工程](https://i.ibb.co/hFMQJbY9/10.png)

2. 从 Q9000 的 U 盘里选择录制的工程文件夹，打开 `.lx` 工程文件。
   *（小提示：建议先把工程文件夹从 Q9000 的 U 盘拷到本地磁盘，免得长时间导出过程中 Q9000 没电。）*

   ![选择 .lx 工程文件](https://i.ibb.co/LXV6SR7b/11.png)

3. 在弹出的对话框里配置处理选项：

   ![单点云工程设置](https://i.ibb.co/y3R8BKh/12.png)

4. 等待加载与处理完成。如果你不需要归档高精度数字孪生资产，勾选 **回环检测** 与 **运动物体滤除** 就够了 —— **平差优化**可以不勾，建图更快。

---

## 3. 导出地图文件

处理完成后，导出两个文件：

- **`.pcd`** —— 3D 点云地图（用于生成 2D `.pgm` + `.yaml` 占据栅格，供全局规划使用）
- **`.bin`** —— Odin 重定位地图

![MindCloud Studio 中处理完成的 3D 点云](https://i.ibb.co/8gH2mPm4/13.png)

### 3.1 导出 `.pcd` 点云

1. 在左侧工程树里右键点云条目，选择 **保存**（`Save`）：

   ![右键点云并点击保存](https://i.ibb.co/Ft90W1Y/14.png)

2. 在保存对话框里把**保存类型**设为 `PCD cloud (*.pcd)`，保存到本地文件夹：

   ![保存为 PCD cloud (*.pcd)](https://i.ibb.co/r28KZbR4/15.png)

### 3.2 导出 `.bin` 重定位地图

1. 顶部菜单栏点 **行业功能 → 导航地图**（`Industry Features → Navigation Map`）：

   ![行业功能 -> 导航地图](https://i.ibb.co/d0gxffrg/16.png)

2. 等待导航地图导出完成（环境越大导出越久）：

   ![导航地图生成进度](https://i.ibb.co/WptMmFtx/17.png)

3. 完成后，导出的 `.bin` 文件在工程目录的 `export_map/` 子文件夹里：

   ![工程目录内的 export_map 文件夹](https://i.ibb.co/XZC96SFV/18.png)

---

## 4. 把 `.pcd` 转成 2D 栅格地图（`.pgm` + `.yaml`）

> [!NOTE]
> 下面的命令以 Unitree Go2 为例（`192.168.123.18`，用户 `unitree`，ROS 1 工作区 `ros_ws/`）。
> 请把 IP、用户名、工作区路径（`ros_ws` 或 `ros2_ws`）以及 `--lidar-height` 换成你自己机器人的值。

### 4.1 把 `.pcd` 与 `.bin` 拷到机器人上

> [!IMPORTANT]
> 仓库根目录下的 `customer_file/`（`/path/to/base_inspection/customer_file/`）默认被 Git 忽略，**新克隆的仓库里没有这个文件夹**。拷贝 `.bin` 重定位地图前，请先**手动创建该文件夹**：
> ```bash
> mkdir -p /path/to/base_inspection/customer_file
> ```

用 U 盘或 `scp` 把导出的 `.pcd` 和 `.bin` 传到机器人：

**方式 A —— 用 U 盘：**

```bash
# 确保仓库根目录下的 customer_file/ 已创建
mkdir -p /path/to/base_inspection/customer_file

# 进入 U 盘挂载目录
cd /media/Upan

# 把 .pcd 与 .bin 移到导航栈对应目录
mv parkinglot.pcd /path/to/base_inspection/ros_ws/src/map_planner/maps/parkinglot.pcd
mv parkinglot.bin /path/to/base_inspection/customer_file/parkinglot.bin

# 确认文件到位
ls /path/to/base_inspection/ros_ws/src/map_planner/maps/parkinglot.pcd
ls /path/to/base_inspection/customer_file/parkinglot.bin
```

**方式 B —— 通过网络用 `scp`：**

```bash
# 先在机器人端创建 customer_file/ 目录（若尚未创建）
ssh unitree@192.168.123.18 "mkdir -p /path/to/base_inspection/customer_file"

# 把点云（.pcd）拷到 map_planner/maps/
scp parkinglot.pcd \
    unitree@192.168.123.18:/path/to/base_inspection/ros_ws/src/map_planner/maps/

# 把重定位地图（.bin）拷到 customer_file/
scp parkinglot.bin \
    unitree@192.168.123.18:/path/to/base_inspection/customer_file/
```

### 4.2 运行 `pcd_to_pgm.py`

运行 `scripts/map_processing/pcd_to_pgm.py`（也可以把它拷到 `map_planner/maps/` 下），把 3D `.pcd` 点云的某个高度切片投影成 2D 占据栅格：

```bash
cd /path/to/base_inspection/ros_ws/src/map_planner/maps

python3 /path/to/base_inspection/scripts/map_processing/pcd_to_pgm.py parkinglot.pcd \
    -o parkinglot \
    -r 0.05 \
    --lidar-height 0.545 \
    --slice-above 0.3 \
    --slice-below 0.2
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `parkinglot.pcd` | *(必填)* | 输入的 `.pcd` 文件路径 |
| `-o` | 输入文件名主干 | 输出文件名前缀；会生成 `<prefix>.pgm` 与 `<prefix>.yaml`。**注意：`.pgm`、`.yaml` 与 `.bin` 三者必须同名。** |
| `-r` | `0.05` | 栅格分辨率（米/格），推荐 `0.05`；地图特别大时可提到 `0.1` 以上 |
| `--lidar-height` | `0.545` | 传感器相对地面的安装高度（米）。`0.545` 是 Go2 的默认值，换机器人要改 |
| `--slice-above` | `0.3` | 传感器**上方**保留为障碍的高度窗口（米） |
| `--slice-below` | `0.2` | 传感器**下方**保留为障碍的高度窗口（米） |

> 上下两个切片的选法与机器人高度、传感器安装位置有关，原理见
> [高级调参指南 §3.1b 点云高度切片](../TUNING_GUIDE.zh-CN.md)。

### 4.3 检查生成的文件

脚本会在 `map_planner/maps/` 下生成两个文件：

```text
parkinglot.pgm    # 2D 占据栅格图像
parkinglot.yaml   # ROS map_server 元数据文件
```

你可以直接查看 `parkinglot.pgm`，或用 GIMP 之类的图像编辑器清掉噪声（见 4.4）。

### 4.4 可选：用 GIMP 手工清理地图

如果原始切片里有需要手动抹掉的瞬时杂物或伪影，用 **GIMP** 打开 `.pgm`：

```bash
sudo apt install gimp
gimp
```

把 `parkinglot.pgm` 拖进 GIMP：

![在 GIMP 中打开 .pgm 地图](https://i.ibb.co/nqysk3Ws/27.png)

- **缩放 / 平移：** 按住 `Ctrl` 滚滚轮缩放；按住鼠标中键拖动平移。
- **铅笔工具：** 在左上工具栏选**铅笔**（把**硬度设为 100**），再从调色板里选纯**白**或纯**黑**：
  - **白** = 可通行的空闲区域
  - **黑** = 障碍
  - **灰** = 未知 / 未探索区域

![GIMP 铅笔工具设置](https://i.ibb.co/1jvJ9k6/28.png)

- **导出：** 点 **文件 → 导出为...**，导出格式选 `.pgm`（raw/二进制 PGM），覆盖 `map_planner/maps/` 下的 `parkinglot.pgm`（`parkinglot.yaml` 不要动）。

---

## 5. 使用 `--map=<地图名>` 启动导航

地图文件准备就绪后，在启动脚本中通过 `--map=<地图名>` 指定要加载的地图：

```bash
bash /path/to/base_inspection/scripts/ons2_go2.sh --map=<地图名>    # 例如 --map=parkinglot
# ROS 2 (SW01)：bash /path/to/base_inspection/scripts/ons2_sw01.sh --map=parkinglot
# Dobot Lumo：  bash /path/to/base_inspection/scripts/ons2_dobot_lumo.sh --map=parkinglot
```

其中，`--map` 用于选中建好的地图名。每张地图包含**三个必须同名的文件**：`<地图名>.yaml`、`<地图名>.pgm` 和 `<地图名>.bin`（例如 `parkinglot.yaml`、`parkinglot.pgm`、`parkinglot.bin`），存放位置如下：

| 文件 | 存放目录 | 说明 |
|---|---|---|
| `<地图名>.yaml` | `/path/to/base_inspection/ros_ws/src/map_planner/maps/`<br/>*（ROS 2 为 `ros2_ws/src/map_planner/maps/`）* | 2D 栅格地图元数据配置 |
| `<地图名>.pgm` | `/path/to/base_inspection/ros_ws/src/map_planner/maps/`<br/>*（ROS 2 为 `ros2_ws/src/map_planner/maps/`）* | 2D 占据栅格地图图像 |
| `<地图名>.bin` | `/path/to/base_inspection/customer_file/` | Odin1 3D 重定位地图（**注意：** 新克隆的仓库下默认没有 `customer_file/` 文件夹，需手动执行 `mkdir -p /path/to/base_inspection/customer_file` 创建） |

启动后的完整巡检与控制流程请参见 [01 · 快速开始](../usage/01_quick_start.zh-CN.md)。
