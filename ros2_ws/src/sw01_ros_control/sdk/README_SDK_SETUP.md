# ASTRALL SDK 说明 / SDK Setup

本目录是 **ASTRALL SDK 1.0.7**（璇玑 SW01 官方 C++ SDK）的入库副本：`include/interface.h` 与
`lib/linux/{x86_64,ARM64}/` 下的动态库已随仓库提交，`catkin_make` / `colcon build` 可直接使用，无需另行拷贝。

> 库文件与官方发布包 `ASTRALL_SDK_1.0.7/C++/lib/linux/` 逐字节一致（md5 见第 7 节）；头文件与官方包代码一致，仅注释语言不同。
> 真机（NX）上的 SDK 版本需在机器人连接后校验，**校验通过前以本副本为准**。

---

## 1. 目录内容

| 内容 | 路径 |
| --- | --- |
| 头文件 | `include/interface.h` |
| 动态库（aarch64，NX 使用） | `lib/linux/ARM64/libASTRALL_SDK.so`（含 `.so.1`、`.so.1.0.7`） |
| 动态库（x86_64，仅开发机验证） | `lib/linux/x86_64/libASTRALL_SDK.so`（含 `.so.1`、`.so.1.0.7`） |

> NX（Jetson Orin/Xavier NX）为 **aarch64 (ARM64)**，必须使用 `lib/linux/ARM64` 下的库；
> 若在 x86_64 开发机上编译验证，则使用 `lib/linux/x86_64`。

## 2. 目录结构

```
sw01_ros_control/sdk/
├── README_SDK_SETUP.md          # 本文件
├── include/
│   └── interface.h              # ASTRALL SDK 头文件
└── lib/
    └── linux/
        ├── ARM64/               # NX 使用
        │   ├── libASTRALL_SDK.so         -> 软链接
        │   ├── libASTRALL_SDK.so.1       -> 软链接
        │   └── libASTRALL_SDK.so.1.0.7   -> 实际库文件
        └── x86_64/              # 仅 x86 开发机验证时使用
            └── libASTRALL_SDK.so*
```

## 3. 升级 SDK 的方法

官方发布新版本时，按下面的方式替换（假设官方 SDK 解压在 `~/ASTRALL_SDK_x.y.z`）。
`ros_ws` 与 `ros2_ws` 下的 `sdk/` 目录内容相同，需要同步替换两份。

```bash
cd ~/catkin_ws/src/sw01_ros_control/sdk

mkdir -p include lib/linux/ARM64 lib/linux/x86_64

cp ~/ASTRALL_SDK_x.y.z/C++/include/interface.h            include/
cp -P ~/ASTRALL_SDK_x.y.z/C++/lib/linux/ARM64/libASTRALL_SDK.so*  lib/linux/ARM64/
cp -P ~/ASTRALL_SDK_x.y.z/C++/lib/linux/x86_64/libASTRALL_SDK.so* lib/linux/x86_64/
```

`cp -P` 用于保留软链接关系（`libASTRALL_SDK.so` -> `libASTRALL_SDK.so.1` -> `libASTRALL_SDK.so.1.0.7`）。
升级后更新第 7 节的 md5 表，并在提交信息中注明 SDK 版本。

### 从 NX 上已有工程中获取

如果 NX 上已经有可用的 SDK（例如某个 demo 工程的 `lib/` 目录）：

```bash
# 在 NX 上执行，先确认库文件架构为 aarch64
file /path/to/demo/lib/libASTRALL_SDK.so
# 期望输出包含: ELF 64-bit LSB shared object, ARM aarch64

cp /path/to/demo/lib/include/interface.h   ~/catkin_ws/src/sw01_ros_control/sdk/include/
cp -P /path/to/demo/lib/libASTRALL_SDK.so* ~/catkin_ws/src/sw01_ros_control/sdk/lib/linux/ARM64/
```

## 4. 使用其它路径（不使用本目录）

如果你想使用其它位置的 SDK，可以在编译时通过 `SW01_SDK_ROOT` 指定 SDK 的 `C++` 目录：

```bash
catkin_make -DSW01_SDK_ROOT=$HOME/ASTRALL_SDK_1.0.7/C++
```

该目录需满足 `include/interface.h` 与 `lib/linux/<arch>/libASTRALL_SDK.so` 的结构。
CMake 同时兼容 `libastrall_sdk.so` 这种小写命名，以及把库直接放在 `lib/` 下的简化结构。

## 5. 验证

```bash
# 配置阶段应打印实际使用的 SDK 路径
catkin_make 2>&1 | grep "ASTRALL SDK"
# -- ASTRALL SDK include : /home/xxx/catkin_ws/src/sw01_ros_control/sdk/include
# -- ASTRALL SDK library : /home/xxx/catkin_ws/src/sw01_ros_control/sdk/lib/linux/ARM64/libASTRALL_SDK.so

# 编译产物应能正确解析到 SDK 动态库（无 "not found"）
ldd ~/catkin_ws/devel/lib/sw01_ros_control/sw01_cmd_vel_bridge | grep ASTRALL
# libASTRALL_SDK.so.1 => /home/xxx/sdk/lib/linux/ARM64/libASTRALL_SDK.so.1 (0x...)
```

CMake 已将 SDK 目录写入可执行文件的 RPATH，**正常情况下无需设置 `LD_LIBRARY_PATH`**。
若你把 SDK 目录移动到了别处，重新执行一次 `catkin_make` 即可更新 RPATH；
或临时使用：

```bash
export LD_LIBRARY_PATH=/新的/sdk/lib/linux/ARM64:$LD_LIBRARY_PATH
```

## 6. 常见错误

| 现象 | 原因与处理 |
| --- | --- |
| CMake 报 `未找到 ASTRALL SDK` | `interface.h` 或 `.so` 未按上述结构放置；检查路径大小写（`ARM64` 而非 `arm64` 亦可，两者都会被搜索） |
| 链接报 `undefined reference to AstrallSdkInit` | 库架构与当前平台不匹配（把 x86_64 库用在 NX 上，或反之）；用 `file` 确认后替换 |
| 运行报 `libASTRALL_SDK.so.1: cannot open shared object file` | SDK 目录被移动或只拷贝了 `libASTRALL_SDK.so` 而缺少 `.so.1`；补齐文件并重新编译 |
| `interface.h` 版本与库不一致导致行为异常 | 头文件与动态库必须来自 **同一个** SDK 版本（本包基于 1.0.7 验证） |

## 7. 真机校验（待执行）

机器人连接后，在 NX 上对当前实际使用的 SDK 库计算 md5，与下表比对。
一致则本副本即为真机版本；不一致则按第 3 节替换为真机上的版本，并更新下表。

| 文件（本仓库副本） | md5 |
| --- | --- |
| `lib/linux/ARM64/libASTRALL_SDK.so.1.0.7` | `67af4e00e8d8d9e21542e2c6ee6c1e6b` |
| `lib/linux/x86_64/libASTRALL_SDK.so.1.0.7` | `3d7d69e806bff30f9e49fd2ce3bf9de9` |

```bash
md5sum <真机上使用的 SDK 路径>/lib/linux/ARM64/libASTRALL_SDK.so.1.0.7
```

- [ ] 真机校验通过，记录校验日期与机器人编号：__________
