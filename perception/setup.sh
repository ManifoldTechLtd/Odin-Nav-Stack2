#!/usr/bin/env bash
# =============================================================================
# Odin-perception 一键安装（ONS2 的可选组件）
#
# 作用：clone 感知仓 → 建隔离 venv → 装依赖 → 下载 Florence-2 权重 → 自检
#
# 本脚本**不会修改导航栈**：感知以两个旁挂进程通过 HTTP(8730) 与导航栈对接，
# 无需重编本仓、无需改 launch、无需动任何导航参数。
#
# 用法：
#   bash perception/setup.sh                 # 默认 main 分支，自动判断平台
#   bash perception/setup.sh --tag v0.1.0    # 钉版本（生产环境推荐）
#   bash perception/setup.sh --dir ~/odin-perception
#   bash perception/setup.sh --cpu           # 强制 CPU 版 torch（无 GPU 平台）
#   bash perception/setup.sh --check-only    # 只跑自检，不安装
#   bash perception/setup.sh --skip-weights  # 先不下载权重（离线环境稍后手动放）
#
# 环境变量：
#   PERCEPTION_REPO   感知仓地址，默认 https://github.com/ManifoldTechLtd/Odin-perception.git
#   PERCEPTION_DIR    安装目录，默认 ~/odin-perception
#   HF_ENDPOINT       权重下载镜像，国内建议 export HF_ENDPOINT=https://hf-mirror.com
#
# 前置：git、python3(>=3.8)、可访问 GitHub（或已把仓库放到本地）
# =============================================================================
set -uo pipefail

PERCEPTION_REPO="${PERCEPTION_REPO:-https://github.com/ManifoldTechLtd/Odin-perception.git}"
TARGET_DIR="${PERCEPTION_DIR:-$HOME/odin-perception}"
TAG=""
FORCE_CPU=0
CHECK_ONLY=0
SKIP_WEIGHTS=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --tag)          TAG="${2:?--tag 需要一个版本号}"; shift 2 ;;
    --dir)          TARGET_DIR="${2:?--dir 需要一个路径}"; shift 2 ;;
    --cpu)          FORCE_CPU=1; shift ;;
    --check-only)   CHECK_ONLY=1; shift ;;
    --skip-weights) SKIP_WEIGHTS=1; shift ;;
    -h|--help)      sed -n '3,26p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（--help 查看用法）" >&2; exit 2 ;;
  esac
done

step() { printf '\n\033[1m[%s]\033[0m %s\n' "$1" "$2"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[33m!\033[0m %s\n' "$*"; }
die()  { printf '\n\033[31m安装中止：\033[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 0. 平台判断
step "0/5" "检测平台"
ARCH="$(uname -m)"
PY="$(command -v python3 || true)"
[ -n "$PY" ] || die "找不到 python3"
PYVER="$("$PY" -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
ok "架构 $ARCH / python3 $PYVER / 目录 $TARGET_DIR"

IS_JETSON=0
if [ "$ARCH" = "aarch64" ] && [ -e /etc/nv_tegra_release ]; then
  IS_JETSON=1
  ok "识别为 Jetson 平台（$(head -1 /etc/nv_tegra_release | cut -c1-40)...）"
fi

# ---------------------------------------------------------------- 1. 取代码
step "1/5" "获取 Odin-perception 源码"
if [ -d "$TARGET_DIR/.git" ]; then
  ok "已存在，尝试更新"
  git -C "$TARGET_DIR" fetch --tags --quiet || warn "拉取失败，继续用现有代码"
  if [ -n "$TAG" ]; then
    git -C "$TARGET_DIR" checkout --quiet "$TAG" && ok "已切到 $TAG"
  fi
else
  if [ -n "$TAG" ]; then
    git clone --branch "$TAG" --depth 1 "$PERCEPTION_REPO" "$TARGET_DIR"
  else
    git clone --depth 1 "$PERCEPTION_REPO" "$TARGET_DIR"
  fi || die "clone 失败：请确认网络可达，或用 PERCEPTION_REPO 指定内网/本地地址"
  ok "已克隆到 $TARGET_DIR"
fi
cd "$TARGET_DIR" || die "无法进入 $TARGET_DIR"

# 若感知仓自带安装入口，交给它做，避免两处维护安装逻辑
HAS_SETUP_SH=0
[ -f scripts/setup.sh ] && HAS_SETUP_SH=1

# ---------------------------------------------------------------- 2. 建环境
step "2/5" "创建隔离 Python 环境"
if [ "$CHECK_ONLY" -eq 1 ]; then
  warn "--check-only：跳过安装，直接自检"
else
  if [ "$HAS_SETUP_SH" -eq 1 ] && [ "$IS_JETSON" -eq 0 ] && [ "$FORCE_CPU" -eq 0 ]; then
    ok "使用感知仓自带 scripts/setup.sh"
    bash scripts/setup.sh || die "感知仓 scripts/setup.sh 执行失败"
  else
    [ -d .venv ] || "$PY" -m venv .venv
    # shellcheck disable=SC1091
    . .venv/bin/activate
    ok "venv: $TARGET_DIR/.venv"
    python -m pip install --quiet --upgrade pip setuptools wheel || warn "pip 自升级失败，继续"
    if [ -f requirements.txt ]; then
      python -m pip install --quiet -r requirements.txt || die "依赖安装失败（离线环境请先 pip download 再 --no-index 安装）"
      ok "requirements.txt 安装完成"
    fi
    # 边缘设备：py3.8 的 pip 不支持 PEP 660，必须用普通安装
    python -m pip install --quiet --no-build-isolation . || die "本包安装失败"
    ok "inspection_perception 安装完成"
  fi
fi

VENV_PY="$TARGET_DIR/.venv/bin/python"
[ -x "$VENV_PY" ] || VENV_PY="$TARGET_DIR/.venv-edge/bin/python"
if [ -x "$VENV_PY" ]; then
  ok "推理解释器: $VENV_PY"
else
  warn "未找到 venv（可能使用系统环境），自检将使用 python3"
  VENV_PY="$PY"
fi

# ---------------------------------------------------------------- 3. GPU 提示
step "3/5" "GPU 检查"
CUDA_OK="$("$VENV_PY" -c 'import torch;print(1 if torch.cuda.is_available() else 0)' 2>/dev/null || echo 0)"
if [ "$CUDA_OK" = "1" ]; then
  ok "CUDA 可用，推理将走 GPU（实测约 1.2 s/条）"
else
  warn "CUDA 不可用，将回退 CPU（可能慢 30 倍以上）"
  if [ "$IS_JETSON" -eq 1 ]; then
    cat <<'EOF'
    Jetson 上要吃到 GPU，需要装 NVIDIA 平台 wheel（按 JetPack 版本选路径）：
      # JetPack 5.1.1 (L4T 35.3.1)
      curl -LO https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
      pip install --no-deps --force-reinstall torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
      pip install --no-deps torchvision==0.15.1
    装完重跑本脚本确认 CUDA 为 True。
EOF
  fi
fi

# ---------------------------------------------------------------- 4. 权重
step "4/5" "Florence-2 权重"
if [ "$SKIP_WEIGHTS" -eq 1 ]; then
  warn "--skip-weights：跳过下载（稍后手动放到 models/florence2-base-ft/）"
elif [ -d models/florence2-base-ft ] && [ -f models/florence2-base-ft/config.json ]; then
  ok "权重已就位"
elif [ -f scripts/download_models.sh ]; then
  if [ -n "${HF_ENDPOINT:-}" ]; then ok "使用镜像 $HF_ENDPOINT"; fi
  bash scripts/download_models.sh florence2-base-ft || warn "权重下载失败，可稍后手动放 models/florence2-base-ft/（约 888MB）"
else
  warn "未找到 download_models.sh，请手动准备 models/florence2-base-ft/"
fi

# ---------------------------------------------------------------- 5. 自检
step "5/5" "自检"
if [ -f scripts/doctor.py ]; then
  "$VENV_PY" scripts/doctor.py || warn "doctor 自检未全绿，按上面的提示处理后重跑"
else
  warn "未找到 scripts/doctor.py"
fi

cat <<EOF

============================================================
安装完成。感知是**可选组件**，导航栈无需任何改动。

下一步（两段式启动，与本导航栈并行）：

  # 终端 A：取图服务（ROS 侧，必须用系统解释器因为它带 rospy）
  source /opt/ros/noetic/setup.bash
  cd $TARGET_DIR
  PYTHONPATH="/usr/lib/python3/dist-packages:.\${PYTHONPATH:+:\$PYTHONPATH}" \\
    /usr/bin/python3 -m inspection_perception.capture.ros1_odin_node --serve \\
    --prefer raw --raw-topic /odin1/image/undistorted --port 8730

  # 终端 B：推理（venv 侧）
  source $TARGET_DIR/.venv/bin/activate
  cd $TARGET_DIR
  perception-run --source http --http-url http://127.0.0.1:8730 \\
    --backend florence2 --model-dir models/florence2-base-ft \\
    --device cuda --dtype float32 \\
    --tasks configs/tasks/demo_florence2.yaml \\
    --annotate-url http://127.0.0.1:8730 --out /tmp/result.json

前置：Odin 驱动需推图 —— control_command.yaml 里 sendrgb: 1 且 sendrgbundistort: 1，
      用 rostopic hz /odin1/image/undistorted 确认有数据。

详细用法见 $TARGET_DIR/README.md（含 ROS1/ROS2 取图、标注发布与常见坑）。
============================================================
EOF
