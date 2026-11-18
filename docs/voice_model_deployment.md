# 本地语音模型：使用与部署（SW01 主机）

本文说明 ONS2 机器人主机上的本地语音链路（语音识别 ASR、语音合成 TTS、LLM 代理）如何部署、如何调用，以及远程主机当前的实际状态。

> 状态说明：这套服务的代码目前只存在于远程主机的备份目录
> `/home/mt/base_inspection.bak_20261117_sru_inrepo/scripts/`，**尚未纳入当前仓库**。
> 下文中的脚本路径均指该备份目录；迁入仓库前请按本文的命令手动运行。

---

## 1. 组件总览

| 组件 | 脚本 | 端口 | 作用 | 依赖 |
|---|---|---|---|---|
| ASR 语音识别 | `scripts/asr_server.py` | 18082 | 把录音转成文字，支持热词增强 | FunASR（SeACo-Paraformer / `paraformer-zh`）、ffmpeg |
| TTS 语音合成 | `scripts/tts_server.py` | 18081 | 把文字合成语音（WAV / Opus） | edge-tts（需要外网）、ffmpeg；离线时回退 espeak-ng |
| LLM 代理 | `scripts/llm_proxy.py` | 18080 | OpenAI 兼容接口，转发到上游并注入 API Key | 上游 LLM 服务（默认智谱，需外网） |
| Hermes 面板 | `hermes dashboard` | 9119 | 语音控制台网页（`/odin-brain`） | `hermes` 命令 |

另有一个**与上述服务无关**的语音能力：控制台自带的喊话（`web_console/intercom.py`）直接在控制台进程里调用 edge-tts，不经过 18081。

仓库里还有一条备选路径：`ros_ws/src/yolo_ros/scripts/object_query_node.py` 使用 Vosk 做离线识别，模型目录默认是 `scripts/voicemodel`。远程主机上**没有安装 vosk，也没有该模型目录**，所以目前不可用。

---

## 2. 远程主机当前状态（实测）

检查对象：`192.168.110.218`（用户 `mt`，Ubuntu 24.04，Python 3.12.3，内存 15 GB，无 NVIDIA GPU）。

| 项目 | 状态 |
|---|---|
| 18080 / 18081 / 18082 / 9119 端口 | 均未监听，服务未运行 |
| `ffmpeg` | 已安装 |
| `espeak-ng` | 已安装 |
| `edge_tts` | 已安装（7.3.1） |
| `funasr` / `modelscope` | **未安装** |
| `vosk` / `sounddevice` | 未安装 |
| `torch` | 已安装 |
| `hermes` 命令 | **未安装** |
| ASR 模型缓存（`~/.cache/modelscope`） | **不存在**，ASR 目前无法启动 |
| `/home/yu/miniconda3`（`start_hermes_web.sh` 写死的 ASR 解释器） | **不存在** |

结论：远程主机上的语音链路**还没有部署完成**。需要先完成第 3 节的步骤。

---

## 3. 部署步骤

### 3.1 系统依赖

```bash
sudo apt-get install -y ffmpeg espeak-ng
```

### 3.2 Python 环境（推荐独立 venv）

Ubuntu 24.04 的系统 Python 受 PEP 668 限制，不能直接 `pip install`。用独立的 venv：

```bash
python3 -m venv /home/mt/voice_venv
/home/mt/voice_venv/bin/pip install --upgrade pip
/home/mt/voice_venv/bin/pip install funasr modelscope edge-tts
```

注意：`asr_server.py` 第一行的解释器是 `/home/yu/miniconda3/bin/python`，`start_hermes_web.sh` 也写死了这个路径。远程主机上没有该路径，因此要么按本文第 4 节直接用 venv 的 python 启动，要么修改启动脚本里的 `ASR_PY`。

### 3.3 ASR 模型

`asr_server.py` 调用的是 `AutoModel(model="paraformer-zh", disable_update=True)`。`disable_update=True` 表示**启动时不会检查更新**，模型必须已经在本地缓存中。因此第一次需要在能访问 ModelScope 的环境里下载一次：

```bash
/home/mt/voice_venv/bin/python -c "from funasr import AutoModel; AutoModel(model='paraformer-zh')"
```

下载完成后模型位于 `~/.cache/modelscope/hub/`。如果主机没有外网，可以在另一台能联网的机器上执行同样的命令，再把 `~/.cache/modelscope/` 整个目录拷贝到主机的相同位置。

### 3.4 TTS 与 LLM 代理

- `tts_server.py` 只需要 `edge-tts` 和 `ffmpeg`。edge-tts 需要访问微软的在线服务；网络不通时会自动回退到 espeak-ng（机械音）。
- `llm_proxy.py` 没有额外依赖，但需要 API Key，见第 5 节。

### 3.5 脚本位置

目前脚本在远程备份目录里。建议在部署前把它们复制到一个稳定的位置，例如 `/home/mt/base_inspection/scripts/`，并用 git 管理，避免备份目录被清理。

---

## 4. 启动与停止

以下命令假设脚本位于 `$SCRIPTS=/home/mt/base_inspection.bak_20261117_sru_inrepo/scripts`，venv 位于 `/home/mt/voice_venv`。

```bash
SCRIPTS=/home/mt/base_inspection.bak_20261117_sru_inrepo/scripts
PY=/home/mt/voice_venv/bin/python

# 1) LLM 代理（先设置 API Key，见第 5 节）
nohup $PY $SCRIPTS/llm_proxy.py --port 18080 > /tmp/llm_proxy.log 2>&1 &

# 2) TTS
nohup $PY $SCRIPTS/tts_server.py --port 18081 > /tmp/tts_server.log 2>&1 &

# 3) ASR（首次加载模型约需数十秒）
nohup $PY $SCRIPTS/asr_server.py --port 18082 > /tmp/asr_server.log 2>&1 &
```

确认启动成功：

```bash
curl -s http://127.0.0.1:18082/health   # {"ok": true, "engine": "funasr-seaco-paraformer-zh"}
curl -s http://127.0.0.1:18081/health   # {"ok": true, "voice": "zh-CN-XiaoxiaoNeural"}
ss -ltn | grep -E ':(18080|18081|18082) '
```

停止：用 `ss -ltnp` 查到 PID 后 `kill`。这三个服务没有 systemd 单元，重启主机后需要重新启动。若要开机自启，可仿照 `scripts/systemd/web-console.service` 为每个服务写一个单元文件（`ExecStart` 使用上面的命令，`User=mt`，`Restart=always`）。

---

## 5. 配置项

### 5.1 环境变量

| 变量 | 默认值 | 用于 | 说明 |
|---|---|---|---|
| `ODIN_LLM_API_KEY` | 无 | `llm_proxy.py` | 上游 LLM 的 API Key，**不要写进仓库** |
| `ODIN_LLM_ENV_FILE` | 见脚本 | `llm_proxy.py` | 未设置 `ODIN_LLM_API_KEY` 时，从该文件读取 Key |
| `ODIN_LLM_UPSTREAM` | `https://open.bigmodel.cn/api/paas/v4` | `llm_proxy.py` | 上游地址 |
| `ODIN_LLM_PROXY_PORT` | `18080` | `llm_proxy.py` | 代理端口 |
| `ODIN_TTS_PORT` | `18081` | `tts_server.py` | TTS 端口 |
| `ODIN_TTS_VOICE` | `zh-CN-XiaoxiaoNeural` | `tts_server.py` | 默认音色 |
| `ODIN_LLM_BASE_URL` / `ODIN_LLM_MODEL` | `glm-4.7-flash` 等 | 机器人侧 LLM 客户端 | 参考 `ros2_ws/src/odin_brain/config/llm.env.example` |

API Key 的推荐做法：写在权限为 `600` 的文件里，用 `ODIN_LLM_ENV_FILE` 指向它，不要写进脚本或仓库。

```bash
install -m 600 /dev/null /home/mt/.config/odin_llm.env
echo 'ODIN_LLM_API_KEY=<你的密钥>' > /home/mt/.config/odin_llm.env
export ODIN_LLM_ENV_FILE=/home/mt/.config/odin_llm.env
```

### 5.2 监听地址

三个服务默认都监听 `0.0.0.0`。如果只在本机使用，建议加 `--host 127.0.0.1`，避免局域网内任何人都能调用：

```bash
$PY $SCRIPTS/asr_server.py --host 127.0.0.1 --port 18082
```

---

## 6. 使用教程

### 6.1 语音识别（ASR）

**方式 A：直接上传音频（推荐）**，请求体为原始音频字节，热词放在请求头里：

```bash
curl -s -X POST http://127.0.0.1:18082/asr \
  -H "Content-Type: audio/wav" \
  -H 'X-Hotwords: ["停一下","回起点","取消任务"]' \
  --data-binary @test.wav
```

**方式 B：JSON + base64**：

```bash
curl -s -X POST http://127.0.0.1:18082/asr \
  -H "Content-Type: application/json" \
  -d "{\"audio_b64\": \"$(base64 -w0 test.wav)\", \"hotwords\": [\"停一下\", \"回起点\"]}"
```

返回：

```json
{"ok": true, "text": "停一下", "raw_text": "停 一下", "latency_ms": 820}
```

说明：

- 输入会自动用 ffmpeg 转成 16 kHz 单声道 WAV，支持 wav / webm / ogg / mp3。
- 热词会与内置热词合并（内置列表在 `DEFAULT_HOTWORDS` 中）。热词只提高识别率，不保证一定识别。
- `text` 已去掉中文字符之间的空格，`raw_text` 是模型原始输出。
- 单次请求体上限 25 MB。

### 6.2 语音合成（TTS）

```bash
curl -s -X POST http://127.0.0.1:18081/tts \
  -H "Content-Type: application/json" \
  -d '{"text": "请让一让，机器狗正在巡检", "voice": "zh-CN-XiaoxiaoNeural"}' \
  -o out.wav
```

- 默认返回 `audio/wav`。只要请求路径中包含字符串 `opus`（不区分大小写），就返回 Opus。
- 合成结果有缓存，重复的句子直接返回缓存文件。
- 服务启动约 3 秒后会预先合成一批导航播报短语，因此启动时需要能访问外网。
- edge-tts 失败会重试一次，仍失败则回退 espeak-ng。

### 6.3 LLM 代理

代理提供 OpenAI 兼容接口，机器人端的客户端把基础地址设为 `http://127.0.0.1:18080/v1`：

```bash
curl -s -X POST http://127.0.0.1:18080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "glm-4.7-flash", "messages": [{"role": "user", "content": "你好"}]}'
```

`GET /v1/models` 返回代理的说明信息。代理会注入 API Key，客户端不需要持有密钥。

### 6.4 Hermes 语音控制台（可选）

`start_hermes_web.sh` 会依次启动上面三个服务，然后执行 `hermes dashboard`，打开 `http://127.0.0.1:9119/odin-brain`。**远程主机上没有 `hermes` 命令**，所以这一步目前不可用。安装 Hermes 之前，请只使用 6.1–6.3 的接口。

---

## 7. 资源占用与性能

- 内存：ASR 模型加载后会占用较多内存，本机 15 GB 可以容纳，但要注意与导航栈、控制台同时运行时的余量。
- CPU：没有 GPU，Paraformer 在 CPU 上推理。
- 延迟：`asr_server.py` 返回的 `latency_ms` 可以直接观察；本文没有给出实测数值，请在部署后自行测量并记录。
- TTS：edge-tts 依赖网络，延迟取决于网络质量，缓存命中后几乎为零。

---

## 8. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `asr_server.py` 启动报 `No module named 'funasr'` | 用错了 Python 解释器 | 使用 `/home/mt/voice_venv/bin/python`，并确认已 `pip install funasr` |
| ASR 启动卡住或报模型下载错误 | 没有缓存模型，且主机无外网 | 按 3.3 在联网机器下载并拷贝 `~/.cache/modelscope` |
| `start_hermes_web.sh` 报找不到 `/home/yu/miniconda3/bin/python` | 脚本写死了路径 | 修改 `ASR_PY` 或先手动启动 ASR |
| TTS 声音是机械音 | edge-tts 失败，回退到 espeak-ng | 检查外网；查看 `/tmp/tts_server.log` |
| `llm_proxy` 返回 401 | API Key 未设置或错误 | 检查 `ODIN_LLM_API_KEY` 或 `ODIN_LLM_ENV_FILE` |
| 端口被占用 | 旧进程还在 | `ss -ltnp \| grep <端口>` 查 PID 后结束 |
| ASR 识别结果很差 | 热词太多或录音采样率/声道不对 | 确认录音质量；ffmpeg 会统一转换格式，但无法修复严重失真的录音 |

---

## 9. 安全注意事项

- 三个服务**没有认证**，且默认监听所有网卡。只能在可信网络中使用，或绑定 `127.0.0.1` 并通过 VPN / 隧道访问。
- `llm_proxy` 持有上游 API Key。代理接口只能在可信网络中开放，否则别人可以借用你的额度。
- 语音数据（录音）会经过 ASR 服务，不要在公开网络上传输。

---

## 10. 与其他组件的关系

- 控制台（`web_console`）的喊话功能是独立实现的，直接调用 edge-tts，不依赖 18081。
- 控制台的 `POST /api/audio/speak_raw` 接收浏览器录音并在机器人扬声器播放，它播放的是录音本身，**不会**调用 ASR 识别。
- ASR 结果如果要驱动导航，需要由上层（Hermes 或自定义节点）把文字映射成控制台或导航的指令。这部分目前没有实现。
