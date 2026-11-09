/**
 * Odin Brain Dashboard Plugin UI (Hermes Web Dashboard)
 * Registers:
 *   1. Main Tab `/odin-brain` ("奥丁大脑控制") — Full voice & text control center
 *   2. Slot `chat:top` — Quick voice input & robot control bar right inside `/chat`
 */
(function () {
  var sdk = window.__HERMES_PLUGIN_SDK__;
  var registry = window.__HERMES_PLUGINS__;
  if (!sdk || !registry) return;

  var React = sdk.React;
  var h = React.createElement;
  var useState = sdk.hooks.useState;
  var useEffect = sdk.hooks.useEffect;
  var useRef = sdk.hooks.useRef;
  var fetchJSON = sdk.fetchJSON;

  function getSpeechRecognitionCtor() {
    if (typeof window === "undefined") return null;
    return window.SpeechRecognition || window.webkitSpeechRecognition || null;
  }

  /**
   * Universal Microphone Recorder using MediaRecorder + WebAudio VAD silence auto-stop.
   * Sends recorded audio blob to `/api/plugins/odin-brain/transcribe` (local FunASR SeACo-Paraformer 18082),
   * working 100% offline in mainland China without Google services!
   */
  function startLocalMicRecording(opts) {
    var onStatus = opts.onStatus || function () {};
    var onResult = opts.onResult || function () {};
    var onError = opts.onError || function () {};

    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      onError("当前浏览器不支持麦克风录音 (navigator.mediaDevices.getUserMedia)");
      return null;
    }

    var state = { stopped: false, recorder: null, stream: null, audioCtx: null, timer: null };

    function stopRecording() {
      if (state.stopped) return;
      state.stopped = true;
      if (state.timer) clearInterval(state.timer);
      try {
        if (state.recorder && state.recorder.state !== "inactive") {
          state.recorder.stop();
        }
      } catch (e) {}
      try {
        if (state.audioCtx) state.audioCtx.close();
      } catch (e) {}
    }

    navigator.mediaDevices
      .getUserMedia({ audio: true })
      .then(function (stream) {
        if (state.stopped) {
          stream.getTracks().forEach(function (t) { t.stop(); });
          return;
        }
        state.stream = stream;
        var chunks = [];
        var rec = new MediaRecorder(stream);
        state.recorder = rec;

        rec.ondataavailable = function (e) {
          if (e.data && e.data.size > 0) chunks.push(e.data);
        };

        rec.onstop = function () {
          stream.getTracks().forEach(function (t) { t.stop(); });
          var blob = new Blob(chunks, { type: rec.mimeType || "audio/webm" });
          if (blob.size < 200) {
            onError("录音过短或为空");
            return;
          }
          onStatus("正在通过本地 FunASR 热词模型识别语音...");
          var reader = new FileReader();
          reader.onloadend = function () {
            var dataUrl = reader.result;
            fetchJSON("/api/plugins/odin-brain/transcribe", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ audio_b64: dataUrl }),
            })
              .then(function (res) {
                if (res && res.ok && res.text) {
                  onResult(res.text, res.latency_ms);
                } else {
                  onError((res && res.error) || "未识别到有效语音，请重试");
                }
              })
              .catch(function (err) {
                onError("ASR 调用失败: " + (err && err.message ? err.message : String(err)));
              });
          };
          reader.readAsDataURL(blob);
        };

        rec.start();
        onStatus("正在录音... 请直接说话（停顿 1.2 秒自动识别，或再次点击按钮结束）");

        // WebAudio energy VAD: auto-stop after speech + 1.2s silence (or max 12s)
        try {
          var AudioCtx = window.AudioContext || window.webkitAudioContext;
          if (AudioCtx) {
            var audioCtx = new AudioCtx();
            state.audioCtx = audioCtx;
            var source = audioCtx.createMediaStreamSource(stream);
            var analyser = audioCtx.createAnalyser();
            analyser.fftSize = 512;
            source.connect(analyser);
            var data = new Uint8Array(analyser.fftSize);
            var heardSpeech = false;
            var silenceTicks = 0;
            var totalTicks = 0;
            state.timer = setInterval(function () {
              totalTicks++;
              if (totalTicks > 120) {
                stopRecording();
                return;
              }
              analyser.getByteTimeDomainData(data);
              var sum = 0;
              for (var i = 0; i < data.length; i++) {
                var v = (data[i] - 128) / 128.0;
                sum += v * v;
              }
              var rms = Math.sqrt(sum / data.length);
              if (rms > 0.035) {
                heardSpeech = true;
                silenceTicks = 0;
              } else if (heardSpeech) {
                silenceTicks++;
                if (silenceTicks >= 12) {
                  // 1.2s of silence after speech -> auto finish!
                  stopRecording();
                }
              }
            }, 100);
          }
        } catch (e) {}
      })
      .catch(function (err) {
        state.stopped = true;
        onError("无法访问麦克风: " + (err && err.message ? err.message : String(err)));
      });

    return { stop: stopRecording };
  }

  // ---------------------------------------------------------------------------
  // Compact Voice & Quick-Control Bar injected at the top of `/chat`
  // ---------------------------------------------------------------------------
  function ChatTopVoiceBar() {
    var _text = useState("");
    var text = _text[0], setText = _text[1];
    var _listening = useState(false);
    var listening = _listening[0], setListening = _listening[1];
    var _busy = useState(false);
    var busy = _busy[0], setBusy = _busy[1];
    var _lastReply = useState("");
    var lastReply = _lastReply[0], setLastReply = _lastReply[1];
    var recRef = useRef(null);

    function runCmd(instruction) {
      if (!instruction || busy) return;
      setBusy(true);
      setLastReply("正在执行: " + instruction + " ...");
      fetchJSON("/api/plugins/odin-brain/command", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ instruction: instruction }),
      })
        .then(function (res) {
          setLastReply((res && res.reply) || "执行完成 (" + ((res && res.state) || "OK") + ")");
        })
        .catch(function (err) {
          setLastReply("调用失败: " + (err && err.message ? err.message : String(err)));
        })
        .finally(function () {
          setBusy(false);
        });
    }

    function toggleMic() {
      if (listening && recRef.current) {
        recRef.current.stop();
        setListening(false);
        return;
      }
      setListening(true);
      recRef.current = startLocalMicRecording({
        onStatus: function (msg) { setLastReply(msg); },
        onResult: function (recognizedText, ms) {
          setListening(false);
          setText(recognizedText);
          setLastReply("🎙️ 识别成功 (" + ms + "ms): “" + recognizedText + "”，正在执行...");
          runCmd(recognizedText);
        },
        onError: function (errMsg) {
          setListening(false);
          setLastReply("⚠️ " + errMsg);
        },
      });
    }

    return h(
      "div",
      {
        style: {
          display: "flex",
          flexWrap: "wrap",
          alignItems: "center",
          gap: "8px",
          padding: "8px 12px",
          marginBottom: "8px",
          borderRadius: "8px",
          background: "rgba(20, 184, 166, 0.08)",
          border: "1px solid rgba(20, 184, 166, 0.3)",
          fontSize: "13px",
        },
      },
      h("strong", { style: { color: "#2dd4bf" } }, "🤖 奥丁大脑语音栏"),
      h(
        "button",
        {
          onClick: toggleMic,
          style: {
            padding: "5px 12px",
            borderRadius: "6px",
            border: "none",
            cursor: "pointer",
            fontWeight: "600",
            background: listening ? "#ef4444" : "#14b8a6",
            color: "#fff",
          },
        },
        listening ? "🔴 正在听...（说完自动执行）" : "🎙️ 点击说指令"
      ),
      h("input", {
        type: "text",
        value: text,
        onChange: function (e) { setText(e.target.value); },
        onKeyDown: function (e) { if (e.key === "Enter") runCmd(text); },
        placeholder: "或直接输入指令：如 冰箱在哪，带我去 / 先去桌子再去冰箱...",
        style: {
          flex: "1",
          minWidth: "220px",
          padding: "5px 10px",
          borderRadius: "6px",
          border: "1px solid rgba(255,255,255,0.2)",
          background: "rgba(0,0,0,0.3)",
          color: "inherit",
        },
      }),
      h(
        "button",
        {
          onClick: function () { runCmd(text); },
          disabled: busy,
          style: {
            padding: "5px 12px",
            borderRadius: "6px",
            border: "1px solid rgba(255,255,255,0.25)",
            cursor: "pointer",
            background: "rgba(255,255,255,0.1)",
            color: "inherit",
          },
        },
        busy ? "执行中..." : "发送给奥丁"
      ),
      h(
        "a",
        {
          href: "/odin-brain",
          style: { color: "#2dd4bf", textDecoration: "underline", marginLeft: "4px" },
        },
        "打开完整控制台 →"
      ),
      lastReply
        ? h(
            "div",
            { style: { width: "100%", color: "#a7f3d0", marginTop: "2px", fontSize: "12px" } },
            "💬 奥丁回复: " + lastReply
          )
        : null
    );
  }

  // ---------------------------------------------------------------------------
  // Full Tab Page `/odin-brain` ("奥丁大脑控制")
  // ---------------------------------------------------------------------------
  function OdinBrainPage() {
    var _status = useState(null);
    var status = _status[0], setStatus = _status[1];
    var _input = useState("");
    var input = _input[0], setInput = _input[1];
    var _listening = useState(false);
    var listening = _listening[0], setListening = _listening[1];
    var _autoSend = useState(true);
    var autoSend = _autoSend[0], setAutoSend = _autoSend[1];
    var _viaHermes = useState(false);
    var viaHermes = _viaHermes[0], setViaHermes = _viaHermes[1];
    var _dryRun = useState(false);
    var dryRun = _dryRun[0], setDryRun = _dryRun[1];
    var _asrMock = useState("");
    var asrMock = _asrMock[0], setAsrMock = _asrMock[1];
    var _busy = useState(false);
    var busy = _busy[0], setBusy = _busy[1];
    var _logs = useState([]);
    var logs = _logs[0], setLogs = _logs[1];
    var recRef = useRef(null);

    function refreshStatus() {
      fetchJSON("/api/plugins/odin-brain/status")
        .then(function (data) { setStatus(data); })
        .catch(function () {});
    }

    useEffect(function () {
      refreshStatus();
    }, []);

    function appendLog(entry) {
      setLogs(function (prev) {
        return [entry].concat(prev.slice(0, 19));
      });
    }

    function sendInstruction(instr, extra) {
      var text = (instr != null ? instr : input).trim();
      if (!text) {
        appendLog({
          ts: new Date().toLocaleTimeString(),
          input: "（空输入）",
          ok: false,
          reply: "请先在输入框输入中文指令（如：去冰箱），或点击左侧绿色按钮用语音说指令。",
          intent: "hint",
          state: "IDLE",
          target: "web_ui",
          missions: [],
          trace: [],
        });
        return;
      }
      if (busy) return;
      setBusy(true);
      appendLog({
        ts: new Date().toLocaleTimeString(),
        input: text,
        ok: true,
        reply: "⏳ 正在下发并执行指令，请稍候...",
        intent: "running",
        state: "RUNNING",
        target: viaHermes ? "hermes_agent" : "odin_brain",
        missions: [],
        trace: ["[web ] 已向 /api/plugins/odin-brain/command 发送请求..."],
      });
      var payload = Object.assign(
        {
          instruction: text,
          dry_run: dryRun,
          via_hermes: viaHermes,
        },
        extra || {}
      );
      if (asrMock.trim() && !payload.simulated_asr_feedback) {
        payload.simulated_asr_feedback = asrMock.trim();
      }
      fetchJSON("/api/plugins/odin-brain/command", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      })
        .then(function (res) {
          var traceList = (res && res.execution_trace) || [];
          if (traceList.length === 0 && res && res.stdout) {
            traceList = String(res.stdout).split("\n");
          }
          setLogs(function (prev) {
            var rest = prev.filter(function (it, idx) {
              return !(idx === 0 && it.state === "RUNNING" && it.input === text);
            });
            return [
              {
                ts: new Date().toLocaleTimeString(),
                input: text,
                ok: !!(res && res.ok),
                reply: (res && (res.reply || res.stdout || res.stderr)) || "执行完成",
                intent: (res && res.intent) || "-",
                state: (res && res.state) || "-",
                target: (res && res.execution_target) || "-",
                missions: (res && res.missions_dispatched) || [],
                trace: traceList,
              },
            ].concat(rest.slice(0, 19));
          });
          refreshStatus();
        })
        .catch(function (err) {
          setLogs(function (prev) {
            var rest = prev.filter(function (it, idx) {
              return !(idx === 0 && it.state === "RUNNING" && it.input === text);
            });
            return [
              {
                ts: new Date().toLocaleTimeString(),
                input: text,
                ok: false,
                reply: "请求异常: " + (err && err.message ? err.message : String(err)),
                intent: "error",
                state: "ERROR",
                target: "-",
                missions: [],
                trace: [],
              },
            ].concat(rest.slice(0, 19));
          });
        })
        .finally(function () {
          setBusy(false);
        });
    }

    function sendControl(action) {
      fetchJSON("/api/plugins/odin-brain/control", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: action }),
      }).then(function (res) {
        appendLog({
          ts: new Date().toLocaleTimeString(),
          input: "[控制] " + action,
          ok: res.ok,
          reply: res.response || res.message || action,
          intent: "control",
          state: action.toUpperCase(),
          target: res.target || "-",
          missions: [],
          trace: [],
        });
        refreshStatus();
      });
    }

    function startOrStopVoice() {
      if (listening && recRef.current) {
        recRef.current.stop();
        setListening(false);
        return;
      }
      setListening(true);
      recRef.current = startLocalMicRecording({
        onStatus: function (msg) {
          setInput(msg);
        },
        onResult: function (recognizedText, ms) {
          setListening(false);
          setInput(recognizedText);
          if (autoSend && recognizedText.trim()) {
            sendInstruction(recognizedText.trim());
          }
        },
        onError: function (errMsg) {
          setListening(false);
          appendLog({
            ts: new Date().toLocaleTimeString(),
            input: "[麦克风语音识别]",
            ok: false,
            reply: errMsg,
            intent: "asr",
            state: "ASR_ERROR",
            target: "local_funasr_18082",
            missions: [],
            trace: [],
          });
        },
      });
    }

    var places = (status && status.places) || [];
    var cardStyle = {
      padding: "16px",
      borderRadius: "10px",
      border: "1px solid rgba(255,255,255,0.12)",
      background: "rgba(255,255,255,0.03)",
      marginBottom: "16px",
    };
    var btnStyle = function (bg, color) {
      return {
        padding: "8px 14px",
        borderRadius: "7px",
        border: "none",
        cursor: "pointer",
        fontWeight: "600",
        fontSize: "13px",
        background: bg || "rgba(255,255,255,0.1)",
        color: color || "#fff",
      };
    };

    var CASE_A_CMD =
      "去桌子靠窗那边，到达之后说话：我带的东西到达请查收。如果你听到有人说“好的”或者“我拿到了”之后你就去冰箱，如果等待20秒没有人给你对应的反馈就直接去工位A，说“货已送达无人领取已返回。”";

    return h(
      "div",
      { style: { maxWidth: "1100px", margin: "0 auto", padding: "20px" } },
      // Header
      h(
        "div",
        { style: { display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "16px" } },
        h(
          "div",
          null,
          h("h2", { style: { fontSize: "22px", fontWeight: "700", margin: "0 0 4px 0" } }, "🤖 奥丁大模型大脑 · 网页语音与导航控制台"),
          h(
            "p",
            { style: { margin: 0, opacity: 0.75, fontSize: "13px" } },
            "后端模型: GLM-4-Flash (智谱 BigModel) | 机器人: Unitree Go2 | 支持网页语音直接说指令、方位问答、多步混合编排与条件分支"
          )
        ),
        h(
          "button",
          { onClick: refreshStatus, style: btnStyle("rgba(255,255,255,0.12)") },
          "🔄 刷新 Go2 状态"
        )
      ),

      // Status Bar Card
      h(
        "div",
        { style: Object.assign({}, cardStyle, { display: "flex", flexWrap: "wrap", gap: "20px", alignItems: "center" }) },
        h(
          "div",
          null,
          h("div", { style: { fontSize: "12px", opacity: 0.65 } }, "Go2 连接地址"),
          h("div", { style: { fontWeight: "600" } }, (status && status.go2_host) || "ODIN_GO2_HOST")
        ),
        h(
          "div",
          null,
          h("div", { style: { fontSize: "12px", opacity: 0.65 } }, "ROS1 导航栈状态"),
          h(
            "div",
            { style: { fontWeight: "600", color: status && status.ros_navigation_stack_online ? "#34d399" : "#fbbf24" } },
            status && status.ros_navigation_stack_online ? "🟢 在线（真实下发）" : "🟡 未启动（自动使用 Go2 真实地点库仿真闭环）"
          )
        ),
        h(
          "div",
          null,
          h("div", { style: { fontSize: "12px", opacity: 0.65 } }, "当前地图 (map_id)"),
          h("div", { style: { fontWeight: "600" } }, (status && status.map_id) || "map_opt")
        ),
        h(
          "div",
          null,
          h("div", { style: { fontSize: "12px", opacity: 0.65 } }, "已标定地点数"),
          h("div", { style: { fontWeight: "600" } }, String(places.length) + " 个地点")
        )
      ),

      // Voice + Text Input Card
      h(
        "div",
        { style: cardStyle },
        h("div", { style: { fontWeight: "600", marginBottom: "10px", fontSize: "15px" } }, "🎙️ 语音 / 文字指令输入"),
        h(
          "div",
          { style: { display: "flex", gap: "10px", marginBottom: "12px", flexWrap: "wrap" } },
          h(
            "button",
            {
              onClick: startOrStopVoice,
              style: {
                padding: "12px 22px",
                borderRadius: "8px",
                border: "none",
                cursor: "pointer",
                fontWeight: "700",
                fontSize: "15px",
                background: listening ? "#ef4444" : "#10b981",
                color: "#fff",
                boxShadow: listening ? "0 0 14px rgba(239,68,68,0.6)" : "none",
              },
            },
            listening ? "🔴 正在听您说话...（点击停止）" : "🎙️ 点击这里用语音说指令"
          ),
          h("input", {
            type: "text",
            value: input,
            onChange: function (e) { setInput(e.target.value); },
            onKeyDown: function (e) { if (e.key === "Enter") sendInstruction(input); },
            placeholder: "点击左侧绿色按钮直接说话，或在此输入中文指令（如：冰箱在哪，带我去）...",
            style: {
              flex: "1",
              minWidth: "280px",
              padding: "10px 14px",
              borderRadius: "8px",
              border: "1px solid rgba(255,255,255,0.22)",
              background: "rgba(0,0,0,0.35)",
              color: "inherit",
              fontSize: "14px",
            },
          }),
          h(
            "button",
            {
              onClick: function () { sendInstruction(input); },
              disabled: busy,
              style: btnStyle("#3b82f6", "#fff"),
            },
            busy ? "⏳ 执行中..." : "🚀 发送执行"
          )
        ),
        // Options row
        h(
          "div",
          { style: { display: "flex", gap: "18px", flexWrap: "wrap", alignItems: "center", fontSize: "13px", opacity: 0.9 } },
          h(
            "label",
            { style: { display: "flex", alignItems: "center", gap: "6px", cursor: "pointer" } },
            h("input", {
              type: "checkbox",
              checked: autoSend,
              onChange: function (e) { setAutoSend(e.target.checked); },
            }),
            "语音说完后自动发送执行"
          ),
          h(
            "label",
            { style: { display: "flex", alignItems: "center", gap: "6px", cursor: "pointer" } },
            h("input", {
              type: "checkbox",
              checked: dryRun,
              onChange: function (e) { setDryRun(e.target.checked); },
            }),
            "仅预览计划 (dry-run，不移动)"
          ),
          h(
            "label",
            { style: { display: "flex", alignItems: "center", gap: "6px", cursor: "pointer" } },
            h("input", {
              type: "checkbox",
              checked: viaHermes,
              onChange: function (e) { setViaHermes(e.target.checked); },
            }),
            "经过 Hermes Agent (GLM-4 Tool Calling) 转发"
          )
        )
      ),

      // Emergency & Mission Control + Calibrated Places Grid
      h(
        "div",
        { style: { display: "grid", gridTemplateColumns: "1fr 1fr", gap: "16px", marginBottom: "16px" } },
        // Left: Places
        h(
          "div",
          { style: Object.assign({}, cardStyle, { marginBottom: 0 }) },
          h("div", { style: { fontWeight: "600", marginBottom: "10px" } }, "📍 Go2 已标定地点库（点击直接去 / 问方位）"),
          places.length === 0
            ? h("div", { style: { opacity: 0.6, fontSize: "13px" } }, "暂无地点")
            : h(
                "div",
                { style: { display: "flex", flexDirection: "column", gap: "8px" } },
                places.map(function (p) {
                  return h(
                    "div",
                    {
                      key: p.name,
                      style: {
                        display: "flex",
                        justifyContent: "space-between",
                        alignItems: "center",
                        padding: "8px 10px",
                        borderRadius: "6px",
                        background: "rgba(255,255,255,0.04)",
                      },
                    },
                    h(
                      "div",
                      null,
                      h("strong", null, p.name),
                      p.aliases && p.aliases.length
                        ? h("span", { style: { fontSize: "12px", opacity: 0.65, marginLeft: "6px" } }, "(" + p.aliases.join("/") + ")")
                        : null,
                      h("div", { style: { fontSize: "11px", opacity: 0.5 } }, "x=" + p.x + ", y=" + p.y + ", yaw=" + p.yaw)
                    ),
                    h(
                      "div",
                      { style: { display: "flex", gap: "6px" } },
                      h(
                        "button",
                        {
                          onClick: function () {
                            setInput("去" + p.name);
                            sendInstruction("去" + p.name);
                          },
                          style: btnStyle("#14b8a6"),
                        },
                        "去这里"
                      ),
                      h(
                        "button",
                        {
                          onClick: function () {
                            var cmd = p.name + "在哪，带我去";
                            setInput(cmd);
                            sendInstruction(cmd);
                          },
                          style: btnStyle("rgba(255,255,255,0.14)"),
                        },
                        "问方位并去"
                      )
                    )
                  );
                })
              )
        ),

        // Right: Control & Preset Scenarios
        h(
          "div",
          { style: Object.assign({}, cardStyle, { marginBottom: 0 }) },
          h("div", { style: { fontWeight: "600", marginBottom: "10px" } }, "🛑 实时运动控制"),
          h(
            "div",
            { style: { display: "flex", gap: "8px", flexWrap: "wrap", marginBottom: "16px" } },
            h("button", { onClick: function () { sendControl("pause"); }, style: btnStyle("#f59e0b") }, "⏸ 暂停 (停一下)"),
            h("button", { onClick: function () { sendControl("resume"); }, style: btnStyle("#10b981") }, "▶️ 继续走"),
            h("button", { onClick: function () { sendControl("cancel"); }, style: btnStyle("#ef4444") }, "🛑 取消任务"),
            h("button", { onClick: function () { sendControl("return_home"); }, style: btnStyle("#6366f1") }, "🏠 返航回起点")
          ),
          h("div", { style: { fontWeight: "600", marginBottom: "8px" } }, "🧪 典型验收场景一键测试"),
          h(
            "div",
            { style: { display: "flex", flexDirection: "column", gap: "7px" } },
            h(
              "button",
              {
                onClick: function () {
                  var c = "先去桌子，喊一声忠诚，然后再去冰箱等待3秒，最后回到工位A";
                  setInput(c);
                  sendInstruction(c);
                },
                style: Object.assign({}, btnStyle("rgba(59,130,246,0.2)"), { textAlign: "left" }),
              },
              "① 混合多步指令：去桌子 → 喊忠诚 → 去冰箱等3秒 → 回工位A"
            ),
            h(
              "button",
              {
                onClick: function () {
                  setInput(CASE_A_CMD);
                  sendInstruction(CASE_A_CMD, { simulated_asr_feedback: "好的，我拿到了", wait_timeout_sec: 5 });
                },
                style: Object.assign({}, btnStyle("rgba(16,185,129,0.2)"), { textAlign: "left" }),
              },
              "② 案例 A（匹配分支）：去桌子靠窗说请查收 → 听到“好的” → 去冰箱"
            ),
            h(
              "button",
              {
                onClick: function () {
                  setInput(CASE_A_CMD);
                  sendInstruction(CASE_A_CMD, { simulated_asr_feedback: "", wait_timeout_sec: 2 });
                },
                style: Object.assign({}, btnStyle("rgba(245,158,11,0.2)"), { textAlign: "left" }),
              },
              "③ 案例 A（超时分支）：去桌子靠窗说请查收 → 等无人应答 → 去工位A汇报"
            ),
            h(
              "button",
              {
                onClick: function () {
                  fetchJSON("/api/plugins/odin-brain/speak", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ text: "留形科技！忠！诚！网页语音控制已就绪！" }),
                  }).then(function (res) {
                    appendLog({
                      ts: new Date().toLocaleTimeString(),
                      input: "[Go2 扬声器播报]",
                      ok: res.ok,
                      reply: "已播报: 留形科技！忠！诚！网页语音控制已就绪！",
                      intent: "speak",
                      state: "SPOKEN",
                      target: res.target || "go2",
                      missions: [],
                      trace: [],
                    });
                  });
                },
                style: Object.assign({}, btnStyle("rgba(168,85,247,0.25)"), { textAlign: "left" }),
              },
              "🔊 Go2 真机扬声器试音：“留形科技！忠！诚！网页语音控制已就绪！”"
            )
          )
        )
      ),

      // Execution Logs
      h(
        "div",
        { style: cardStyle },
        h("div", { style: { fontWeight: "600", marginBottom: "10px" } }, "📜 实时执行结果与决策轨迹"),
        logs.length === 0
          ? h("div", { style: { opacity: 0.6, fontSize: "13px" } }, "尚未发送指令。点击上方「🎙️ 点击这里用语音说指令」或任一测试按钮开始。")
          : h(
              "div",
              { style: { display: "flex", flexDirection: "column", gap: "10px" } },
              logs.map(function (item, idx) {
                return h(
                  "div",
                  {
                    key: idx,
                    style: {
                      padding: "12px",
                      borderRadius: "8px",
                      background: "rgba(0,0,0,0.3)",
                      borderLeft: "4px solid " + (item.ok ? "#10b981" : "#ef4444"),
                    },
                  },
                  h(
                    "div",
                    { style: { display: "flex", justifyContent: "space-between", fontSize: "12px", opacity: 0.7, marginBottom: "4px" } },
                    h("span", null, "🕒 " + item.ts + " | 意图: " + item.intent + " | 状态: " + item.state),
                    h("span", null, "执行端: " + item.target)
                  ),
                  h("div", { style: { fontWeight: "600", marginBottom: "4px" } }, "🗣 输入: “" + item.input + "”"),
                  h("div", { style: { color: "#6ee7b7", marginBottom: "6px" } }, "🤖 奥丁回复: " + (item.reply || "（无文本回复）")),
                  item.missions && item.missions.length > 0
                    ? h(
                        "div",
                        { style: { fontSize: "12px", color: "#93c5fd", marginBottom: "4px" } },
                        "🧭 下发航点序列: " + JSON.stringify(item.missions)
                      )
                    : null,
                  item.trace && item.trace.length > 0
                    ? h(
                        "pre",
                        {
                          style: {
                            margin: 0,
                            padding: "8px",
                            borderRadius: "6px",
                            background: "rgba(0,0,0,0.45)",
                            fontSize: "12px",
                            overflowX: "auto",
                            whiteSpace: "pre-wrap",
                          },
                        },
                        item.trace.join("\n")
                      )
                    : null
                );
              })
            )
      )
    );
  }

  registry.register("odin-brain", OdinBrainPage);
  registry.registerSlot("odin-brain", "chat:top", ChatTopVoiceBar);
})();
