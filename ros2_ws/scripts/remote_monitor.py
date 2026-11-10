#!/usr/bin/env python3
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""ONS2 Remote Web Monitor, On-Demand Camera & Audio Intercom Server.

Features:
  1. Safety Monitor (StallDetector): Alerts the remote web console when the robot
     stops or creeps in place (net displacement radius < 0.20m) for > 30 seconds.
  2. Remote Intercom (AudioIntercom): Supports both live browser microphone recording
     and Text-to-Speech (TTS + preset phrases) played on the robot's speaker.
  3. On-Demand Camera (OnDemandImageGate): Dynamically subscribes/unsubscribes to
     `/odin1/image/compressed` only when requested by the web console (default OFF
     for zero 4G bandwidth consumption).
  4. Single-Port Dual HTTP/HTTPS Server: Auto-detects TLS ClientHello (0x16) on a
     single port (default 18088) so NAT traversal only needs 1 port for both HTTP
     and HTTPS (enabling browser microphone permissions over public IPs).
"""

import argparse
import asyncio
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import shutil
import socket
import ssl
import subprocess
import tempfile
import threading
import time
from typing import Callable, Optional, Tuple
from urllib.parse import parse_qs, urlparse


class StallDetector:
    """Detects robot complete stop or creeping-in-place over a sliding time window."""

    ACTIVE_MISSION_STATES = frozenset({
        "NAVIGATING",
        "RETURNING_HOME",
        "ALIGNING",
        "RECOVERING",
        "EXECUTING",
        "MOVING",
    })

    def __init__(
        self,
        window_sec: float = 30.0,
        stall_radius_m: float = 0.20,
        cmd_active_hold_sec: float = 10.0,
        odom_stale_sec: float = 10.0,
    ):
        self.window_sec = float(window_sec)
        self.stall_radius_m = float(stall_radius_m)
        self.cmd_active_hold_sec = float(cmd_active_hold_sec)
        self.odom_stale_sec = float(odom_stale_sec)

        self._lock = threading.Lock()
        self.mission_state = "UNKNOWN"
        self.distance_to_target_m: Optional[float] = None
        self.progress_percent: Optional[float] = None
        self.current_waypoint_idx: Optional[int] = None
        self.total_waypoints: Optional[int] = None
        self.force_monitor = False

        self.poses = deque()  # entries: (t, x, y, vx, vy, vyaw)
        self.last_odom: Optional[dict] = None
        self.last_cmd_vel: Optional[dict] = None
        self.last_nonzero_cmd_at: Optional[float] = None

        self.active_since: Optional[float] = None
        self.stall_anchor: Optional[Tuple[float, float, float]] = None  # (t, x, y)
        self.muted_until: float = 0.0

    def set_config(self, window_sec: Optional[float] = None, stall_radius_m: Optional[float] = None) -> None:
        with self._lock:
            if window_sec is not None and window_sec >= 5.0:
                self.window_sec = float(window_sec)
            if stall_radius_m is not None and stall_radius_m > 0.01:
                self.stall_radius_m = float(stall_radius_m)

    def set_force_monitor(self, enabled: bool, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            self.force_monitor = bool(enabled)
            if self.force_monitor and self.active_since is None:
                self.active_since = now
                if self.last_odom is not None:
                    self.stall_anchor = (now, self.last_odom["x"], self.last_odom["y"])
            elif not self.force_monitor and not self._is_auto_active_locked(now):
                self.active_since = None
                self.stall_anchor = None

    def acknowledge_alarm(self, mute_sec: float = 60.0, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            self.muted_until = now + max(0.0, float(mute_sec))

    def update_mission_state(
        self,
        state: str,
        distance_to_target_m: Optional[float] = None,
        progress_percent: Optional[float] = None,
        current_waypoint_idx: Optional[int] = None,
        total_waypoints: Optional[int] = None,
        now: Optional[float] = None,
    ) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            self.mission_state = (state or "UNKNOWN").strip().upper()
            if distance_to_target_m is not None:
                self.distance_to_target_m = float(distance_to_target_m)
            if progress_percent is not None:
                self.progress_percent = float(progress_percent)
            if current_waypoint_idx is not None:
                self.current_waypoint_idx = int(current_waypoint_idx)
            if total_waypoints is not None:
                self.total_waypoints = int(total_waypoints)
            if self._is_monitoring_active_locked(now):
                if self.active_since is None:
                    self.active_since = now
                    if self.last_odom is not None:
                        self.stall_anchor = (now, self.last_odom["x"], self.last_odom["y"])
            else:
                self.active_since = None
                self.stall_anchor = None

    def update_cmd_vel(self, vx: float, vy: float, vyaw: float, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            speed = math.hypot(float(vx), float(vy))
            self.last_cmd_vel = {
                "t": now,
                "vx": float(vx),
                "vy": float(vy),
                "vyaw": float(vyaw),
            }
            if speed > 0.02 or abs(float(vyaw)) > 0.05:
                self.last_nonzero_cmd_at = now
                if self.active_since is None:
                    self.active_since = now
                    if self.last_odom is not None:
                        self.stall_anchor = (now, self.last_odom["x"], self.last_odom["y"])

    def update_odom(
        self,
        x: float,
        y: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
        yaw: float = 0.0,
        now: Optional[float] = None,
    ) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            x, y = float(x), float(y)
            vx, vy, vyaw, yaw = float(vx), float(vy), float(vyaw), float(yaw)
            self.last_odom = {
                "t": now,
                "x": x,
                "y": y,
                "yaw": yaw,
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
            }
            self.poses.append((now, x, y, vx, vy, vyaw))
            cutoff = now - max(self.window_sec * 2.0, 120.0)
            while self.poses and self.poses[0][0] < cutoff:
                self.poses.popleft()

            if self._is_monitoring_active_locked(now):
                if self.active_since is None:
                    self.active_since = now
                if self.stall_anchor is None:
                    self.stall_anchor = (now, x, y)
                else:
                    dist_from_anchor = math.hypot(x - self.stall_anchor[1], y - self.stall_anchor[2])
                    if dist_from_anchor >= self.stall_radius_m:
                        self.stall_anchor = (now, x, y)

    def _is_auto_active_locked(self, now: float) -> bool:
        if self.mission_state in self.ACTIVE_MISSION_STATES:
            return True
        if self.last_nonzero_cmd_at is not None and (now - self.last_nonzero_cmd_at) <= self.cmd_active_hold_sec:
            return True
        return False

    def _is_monitoring_active_locked(self, now: float) -> bool:
        return self.force_monitor or self._is_auto_active_locked(now)

    def evaluate(self, now: Optional[float] = None) -> dict:
        now = time.time() if now is None else float(now)
        with self._lock:
            active = self._is_monitoring_active_locked(now)
            if not active:
                self.active_since = None
                self.stall_anchor = None
                return self._build_result_locked(
                    now=now,
                    monitoring_active=False,
                    alarm=False,
                    alarm_type="NONE",
                    alarm_message="待机中（导航启动后自动监控）",
                    stall_duration_sec=0.0,
                    max_displacement_m=0.0,
                    path_length_m=0.0,
                )

            if self.active_since is None:
                self.active_since = now

            window_start = now - self.window_sec
            window_poses = [p for p in self.poses if p[0] >= window_start]

            odom_age = (now - self.last_odom["t"]) if self.last_odom else None
            if not window_poses or (odom_age is not None and odom_age > self.odom_stale_sec):
                active_dur = now - self.active_since
                is_alarm = active_dur >= self.window_sec
                return self._build_result_locked(
                    now=now,
                    monitoring_active=True,
                    alarm=is_alarm,
                    alarm_type="ODOM_STALE" if is_alarm else "NONE",
                    alarm_message=(
                        f"警告：里程计数据断流或超时已达 {active_dur:.1f}s！"
                        if is_alarm
                        else f"等待里程计数据（已监控 {active_dur:.1f}s）"
                    ),
                    stall_duration_sec=active_dur,
                    max_displacement_m=0.0,
                    path_length_m=0.0,
                )

            x0, y0 = window_poses[0][1], window_poses[0][2]
            max_disp = 0.0
            path_len = 0.0
            max_speed = 0.0
            max_yaw_rate = 0.0
            prev_x, prev_y = x0, y0

            for _, px, py, pvx, pvy, pvyaw in window_poses:
                d = math.hypot(px - x0, py - y0)
                if d > max_disp:
                    max_disp = d
                path_len += math.hypot(px - prev_x, py - prev_y)
                prev_x, prev_y = px, py
                spd = math.hypot(pvx, pvy)
                if spd > max_speed:
                    max_speed = spd
                if abs(pvyaw) > max_yaw_rate:
                    max_yaw_rate = abs(pvyaw)

            # Calculate how long the robot has stayed within stall_radius_m
            anchor_age = (now - self.stall_anchor[0]) if self.stall_anchor else (now - window_poses[0][0])
            window_span = now - window_poses[0][0]
            effective_stall_sec = max(anchor_age, window_span if max_disp < self.stall_radius_m else 0.0)
            active_duration = now - self.active_since
            stall_duration_sec = min(active_duration, effective_stall_sec)

            if active_duration >= self.window_sec and max_disp < self.stall_radius_m:
                is_creeping = (path_len >= 0.04) or (max_speed >= 0.03) or (max_yaw_rate >= 0.08)
                alarm_type = "CREEPING" if is_creeping else "STOPPED"
                label = "原地蠕动/受阻" if is_creeping else "停止不前"
                msg = (
                    f"警报：机械狗已{label} {stall_duration_sec:.1f} 秒！"
                    f"（近 {self.window_sec:.0f}s 净位移仅 {max_disp:.2f}m < 阈值 {self.stall_radius_m:.2f}m，"
                    f"累计轨迹 {path_len:.2f}m）"
                )
                return self._build_result_locked(
                    now=now,
                    monitoring_active=True,
                    alarm=True,
                    alarm_type=alarm_type,
                    alarm_message=msg,
                    stall_duration_sec=stall_duration_sec,
                    max_displacement_m=max_disp,
                    path_length_m=path_len,
                )

            return self._build_result_locked(
                now=now,
                monitoring_active=True,
                alarm=False,
                alarm_type="NONE",
                alarm_message=f"运行正常（近 {min(window_span, self.window_sec):.0f}s 净位移 {max_disp:.2f}m）",
                stall_duration_sec=stall_duration_sec if max_disp < self.stall_radius_m else 0.0,
                max_displacement_m=max_disp,
                path_length_m=path_len,
            )

    def _build_result_locked(
        self,
        now: float,
        monitoring_active: bool,
        alarm: bool,
        alarm_type: str,
        alarm_message: str,
        stall_duration_sec: float,
        max_displacement_m: float,
        path_length_m: float,
    ) -> dict:
        odom_age = (now - self.last_odom["t"]) if self.last_odom else None
        return {
            "monitoring_active": monitoring_active,
            "force_monitor": self.force_monitor,
            "mission_state": self.mission_state,
            "distance_to_target_m": self.distance_to_target_m,
            "progress_percent": self.progress_percent,
            "current_waypoint_idx": self.current_waypoint_idx,
            "total_waypoints": self.total_waypoints,
            "alarm": alarm,
            "alarm_type": alarm_type,
            "alarm_message": alarm_message,
            "buzzer_muted": now < self.muted_until,
            "muted_remaining_sec": max(0.0, round(self.muted_until - now, 1)),
            "window_sec": self.window_sec,
            "stall_radius_m": self.stall_radius_m,
            "stall_duration_sec": round(stall_duration_sec, 1),
            "max_displacement_m": round(max_displacement_m, 3),
            "path_length_m": round(path_length_m, 3),
            "odom": self.last_odom,
            "odom_age_sec": round(odom_age, 2) if odom_age is not None else None,
            "cmd_vel": self.last_cmd_vel,
        }


class OnDemandImageGate:
    """Controls dynamic ROS 2 subscription to /odin1/image/compressed with 4G JPEG downscaling."""

    def __init__(
        self,
        subscribe_fn: Callable[[], None],
        unsubscribe_fn: Callable[[], None],
        default_fps: float = 2.0,
        max_width: int = 640,
        jpeg_quality: int = 55,
        idle_timeout_sec: float = 300.0,
    ):
        self._subscribe_fn = subscribe_fn
        self._unsubscribe_fn = unsubscribe_fn
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self.enabled = False
        self.fps = max(1.0, min(float(default_fps), 3.0))
        self.max_width = int(max_width)
        self.jpeg_quality = int(jpeg_quality)
        self.idle_timeout_sec = float(idle_timeout_sec)
        self.latest_jpeg: Optional[bytes] = None
        self.latest_seq: int = 0
        self.latest_stamp: float = 0.0
        self.last_client_pull_at: float = 0.0
        self.total_frames: int = 0
        self.total_bytes: int = 0

    def _compress_for_4g(self, raw_jpeg: bytes) -> bytes:
        # Only recompress if frame is larger than 35 KB
        if len(raw_jpeg) <= 35 * 1024 or self.max_width <= 0:
            return raw_jpeg
        try:
            import io
            from PIL import Image

            with Image.open(io.BytesIO(raw_jpeg)) as im:
                im = im.convert("RGB")
                w, h = im.size
                if w > self.max_width:
                    new_h = max(1, int(round(h * (self.max_width / float(w)))))
                    im = im.resize((self.max_width, new_h), Image.Resampling.BILINEAR)
                out = io.BytesIO()
                im.save(out, format="JPEG", quality=self.jpeg_quality, optimize=True)
                return out.getvalue()
        except Exception:
            return raw_jpeg

    def set_enabled(
        self,
        enabled: bool,
        fps: Optional[float] = None,
        now: Optional[float] = None,
    ) -> dict:
        now = time.time() if now is None else float(now)
        call_sub = False
        call_unsub = False
        with self._lock:
            if fps is not None:
                self.fps = max(1.0, min(float(fps), 3.0))
            target = bool(enabled)
            if target and not self.enabled:
                self.enabled = True
                self.last_client_pull_at = now
                call_sub = True
            elif target and self.enabled:
                self.last_client_pull_at = now
            elif not target and self.enabled:
                self.enabled = False
                self.latest_jpeg = None
                call_unsub = True
                self._cond.notify_all()
        if call_sub:
            self._subscribe_fn()
        elif call_unsub:
            self._unsubscribe_fn()
        return self.get_status()

    def on_compressed_frame(self, jpeg_bytes: bytes, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else float(now)
        auto_unsub = False
        with self._lock:
            if not self.enabled or not jpeg_bytes:
                return False
            if (
                self.idle_timeout_sec > 0.0
                and self.last_client_pull_at > 0.0
                and (now - self.last_client_pull_at) > self.idle_timeout_sec
            ):
                self.enabled = False
                self.latest_jpeg = None
                auto_unsub = True
                self._cond.notify_all()
            else:
                min_interval = 1.0 / max(0.5, self.fps)
                if self.latest_stamp > 0.0 and (now - self.latest_stamp) < min_interval:
                    return False

        if auto_unsub:
            self._unsubscribe_fn()
            return False

        compressed = self._compress_for_4g(bytes(jpeg_bytes))
        with self._lock:
            if not self.enabled:
                return False
            self.latest_jpeg = compressed
            self.latest_seq += 1
            self.latest_stamp = now
            self.total_frames += 1
            self.total_bytes += len(self.latest_jpeg)
            self._cond.notify_all()
            return True

    def get_latest_frame(self, now: Optional[float] = None) -> Tuple[Optional[bytes], int, float]:
        now = time.time() if now is None else float(now)
        with self._lock:
            if not self.enabled:
                return None, self.latest_seq, 0.0
            self.last_client_pull_at = now
            return self.latest_jpeg, self.latest_seq, self.latest_stamp

    def wait_for_next_frame(self, last_seq: int, timeout: float = 2.0) -> Tuple[Optional[bytes], int, float]:
        with self._cond:
            if not self.enabled:
                return None, self.latest_seq, 0.0
            if self.latest_seq <= last_seq:
                self._cond.wait(timeout=timeout)
            if not self.enabled:
                return None, self.latest_seq, 0.0
            return self.latest_jpeg, self.latest_seq, self.latest_stamp

    def get_status(self) -> dict:
        with self._lock:
            age = (time.time() - self.latest_stamp) if self.latest_stamp > 0 else None
            return {
                "topic": "/odin1/image/compressed",
                "enabled": self.enabled,
                "fps": round(self.fps, 1),
                "frame_seq": self.latest_seq,
                "frame_size_bytes": len(self.latest_jpeg) if self.latest_jpeg else 0,
                "frame_age_sec": round(age, 2) if age is not None else None,
                "total_frames_sent": self.total_frames,
                "total_kb_sent": round(self.total_bytes / 1024.0, 1),
            }


class AudioIntercom:
    """Handles remote microphone audio playback and Text-to-Speech on the robot."""

    PRESET_PHRASES = (
        "请让一让，机器狗正在巡检",
        "请注意安全，保持距离",
        "机器狗受阻，请协助让行",
        "正在执行导航任务，请勿触碰",
        "收到，正在处理",
        "谢谢配合",
    )

    def __init__(
        self,
        cache_dir: Optional[str] = None,
        default_voice: str = "zh-CN-XiaoxiaoNeural",
        custom_play_fn: Optional[Callable[[str, dict], None]] = None,
        custom_tts_fn: Optional[Callable[[str, str], None]] = None,
    ):
        self.cache_dir = cache_dir or os.path.join(tempfile.gettempdir(), "ons2_intercom_cache")
        os.makedirs(self.cache_dir, exist_ok=True)
        self.default_voice = default_voice
        self._custom_play_fn = custom_play_fn
        self._custom_tts_fn = custom_tts_fn
        self._play_lock = threading.Lock()
        self.last_spoken_at: Optional[float] = None
        self.last_spoken_summary: str = ""
        self.play_count: int = 0

    def speak_tts(self, text: str, volume: int = 100, voice: Optional[str] = None) -> dict:
        clean_text = (text or "").strip()
        if not clean_text:
            raise ValueError("播报文字不能为空")
        if len(clean_text) > 200:
            clean_text = clean_text[:200]
        voice = voice or self.default_voice
        volume = max(20, min(int(volume), 250))

        key = hashlib.sha1(f"{voice}|{clean_text}".encode("utf-8")).hexdigest()[:16]
        base_wav = os.path.join(self.cache_dir, f"tts_{key}.wav")

        if not (os.path.isfile(base_wav) and os.path.getsize(base_wav) > 256):
            if self._custom_tts_fn is not None:
                self._custom_tts_fn(clean_text, base_wav)
            else:
                self._synthesize_to_wav(clean_text, voice, base_wav)

        play_wav = self._apply_volume_if_needed(base_wav, volume)
        meta = {"mode": "tts", "text": clean_text, "volume": volume}
        self._play_wav_file(play_wav, meta)
        return {
            "ok": True,
            "mode": "tts",
            "text": clean_text,
            "volume": volume,
            "play_count": self.play_count,
        }

    def speak_raw_audio(self, raw_bytes: bytes, mime_or_ext: str = "webm", volume: int = 100) -> dict:
        if not raw_bytes or len(raw_bytes) < 64:
            raise ValueError("录音数据为空或过短")
        volume = max(20, min(int(volume), 250))
        ext = "webm"
        lower = (mime_or_ext or "").lower()
        for cand in ("wav", "ogg", "mp4", "m4a", "mp3", "aac", "webm"):
            if cand in lower:
                ext = cand
                break

        stamp = int(time.time() * 1000)
        in_path = os.path.join(self.cache_dir, f"mic_{stamp}_in.{ext}")
        out_wav = os.path.join(self.cache_dir, f"mic_{stamp}_out.wav")
        try:
            with open(in_path, "wb") as f:
                f.write(raw_bytes)
            if self._custom_play_fn is not None and not shutil.which("ffmpeg"):
                out_wav = in_path
            else:
                vol_ratio = volume / 100.0
                subprocess.run(
                    [
                        "ffmpeg", "-y", "-loglevel", "error",
                        "-i", in_path,
                        "-af", f"volume={vol_ratio:.2f}",
                        "-ar", "16000", "-ac", "1", "-f", "wav",
                        out_wav,
                    ],
                    check=True,
                    timeout=15,
                )
            meta = {"mode": "mic", "bytes": len(raw_bytes), "volume": volume}
            self._play_wav_file(out_wav, meta)
            return {
                "ok": True,
                "mode": "mic",
                "bytes": len(raw_bytes),
                "volume": volume,
                "play_count": self.play_count,
            }
        finally:
            for p in (in_path, out_wav):
                if p and os.path.isfile(p) and self._custom_play_fn is None:
                    try:
                        os.remove(p)
                    except OSError:
                        pass

    def _synthesize_to_wav(self, text: str, voice: str, out_wav: str) -> None:
        mp3_path = out_wav[:-4] + ".mp3"
        edge_ok = False
        try:
            import edge_tts

            async def _run_edge():
                await edge_tts.Communicate(text, voice).save(mp3_path)

            asyncio.run(_run_edge())
            if os.path.isfile(mp3_path) and os.path.getsize(mp3_path) > 256:
                subprocess.run(
                    [
                        "ffmpeg", "-y", "-loglevel", "error",
                        "-i", mp3_path,
                        "-ar", "16000", "-ac", "1", "-f", "wav",
                        out_wav,
                    ],
                    check=True,
                    timeout=20,
                )
                edge_ok = True
        except Exception:
            edge_ok = False
        finally:
            if os.path.isfile(mp3_path):
                try:
                    os.remove(mp3_path)
                except OSError:
                    pass

        if edge_ok and os.path.isfile(out_wav):
            return

        if shutil.which("espeak-ng"):
            subprocess.run(
                ["espeak-ng", "-v", "cmn", "-s", "150", "-w", out_wav, text],
                check=True,
                timeout=10,
            )
            return

        if shutil.which("ffmpeg"):
            # Fallback attention chime if no offline TTS binary is installed
            subprocess.run(
                [
                    "ffmpeg", "-y", "-loglevel", "error",
                    "-f", "lavfi", "-i", "sine=frequency=880:duration=0.6",
                    "-ar", "16000", "-ac", "1", out_wav,
                ],
                check=True,
                timeout=10,
            )
            return

        raise RuntimeError("未找到可用的语音合成器 (edge-tts / espeak-ng / ffmpeg)")

    def _apply_volume_if_needed(self, base_wav: str, volume: int) -> str:
        if volume == 100 or not shutil.which("ffmpeg") or self._custom_play_fn is not None:
            return base_wav
        scaled_wav = base_wav[:-4] + f"_v{volume}.wav"
        if os.path.isfile(scaled_wav) and os.path.getsize(scaled_wav) > 256:
            return scaled_wav
        vol_ratio = volume / 100.0
        subprocess.run(
            [
                "ffmpeg", "-y", "-loglevel", "error",
                "-i", base_wav,
                "-af", f"volume={vol_ratio:.2f}",
                "-ar", "16000", "-ac", "1", "-f", "wav",
                scaled_wav,
            ],
            check=True,
            timeout=10,
        )
        return scaled_wav

    def _detect_usb_card_indices(self) -> list:
        indices = []
        try:
            if os.path.isfile("/proc/asound/cards"):
                with open("/proc/asound/cards", "r", encoding="utf-8", errors="ignore") as f:
                    for line in f:
                        if "USB-Audio" in line or "USB Audio" in line:
                            parts = line.strip().split()
                            if parts and parts[0].isdigit():
                                indices.append(int(parts[0]))
        except Exception:
            pass
        return indices

    def _play_wav_file(self, wav_path: str, meta: dict) -> None:
        with self._play_lock:
            self.play_count += 1
            self.last_spoken_at = time.time()
            self.last_spoken_summary = meta.get("text") or f"麦克风原声 ({meta.get('bytes', 0)}B)"
            if self._custom_play_fn is not None:
                self._custom_play_fn(wav_path, meta)
                return

            if shutil.which("amixer"):
                for ctl in ("Master", "Headphone", "Speaker", "PCM"):
                    subprocess.run(
                        ["amixer", "-c", "0", "sset", ctl, "95%", "unmute"],
                        capture_output=True,
                        timeout=3,
                    )

            players = []
            usb_cards = self._detect_usb_card_indices()
            if shutil.which("aplay"):
                for c in usb_cards:
                    players.append(["aplay", "-q", "-D", f"plughw:{c},0", wav_path])
            if shutil.which("paplay"):
                players.append(["paplay", wav_path])
            if shutil.which("pw-play"):
                players.append(["pw-play", wav_path])
            if shutil.which("aplay"):
                players.append(["aplay", "-q", "-D", "plughw:0,0", wav_path])
                players.append(["aplay", "-q", wav_path])
            if shutil.which("ffplay"):
                players.append(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", wav_path])

            last_err = None
            for cmd in players:
                try:
                    env = os.environ.copy()
                    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
                    env.setdefault("SDL_AUDIODRIVER", "alsa")
                    env.setdefault("AUDIODEV", "plughw:0,0")
                    res = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)
                    if res.returncode == 0:
                        return
                    last_err = res.stderr.strip() or f"exit {res.returncode}"
                except Exception as exc:
                    last_err = str(exc)

            raise RuntimeError(f"音频播放失败 ({last_err or '无可用播放器'})")

    def get_status(self) -> dict:
        return {
            "play_count": self.play_count,
            "last_spoken_at": self.last_spoken_at,
            "last_spoken_summary": self.last_spoken_summary,
            "presets": list(self.PRESET_PHRASES),
        }


class NavigationAndTeleopController:
    """Manages Emergency Stop, Navigation Mission Controls (Pause/Resume/Skip/Cancel), and Manual Teleop Takeover."""

    def __init__(
        self,
        call_service_fn: Optional[Callable[[str], Tuple[bool, str]]] = None,
        publish_cmd_vel_fn: Optional[Callable[[float, float, float], None]] = None,
        publish_neupan_stop_fn: Optional[Callable[[], None]] = None,
        publish_estop_fn: Optional[Callable[[bool], None]] = None,
        cmd_timeout_sec: float = 0.6,
        max_linear_vel: float = 0.6,
        max_angular_vel: float = 0.8,
    ):
        self._call_service_fn = call_service_fn or (lambda srv: (True, "noop"))
        self._publish_cmd_vel_fn = publish_cmd_vel_fn or (lambda vx, vy, vyaw: None)
        self._publish_neupan_stop_fn = publish_neupan_stop_fn or (lambda: None)
        self._publish_estop_fn = publish_estop_fn or (lambda flag: None)
        self.cmd_timeout_sec = float(cmd_timeout_sec)
        self.max_linear_vel = float(max_linear_vel)
        self.max_angular_vel = float(max_angular_vel)

        self._lock = threading.Lock()
        self.estop_locked = False
        self.teleop_active = False
        self.target_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_vel_at: float = 0.0
        self.last_action_message: str = "就绪"
        self.last_action_at: Optional[float] = None

    @staticmethod
    def _clamp(val: float, limit: float) -> float:
        try:
            v = float(val)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(v):
            return 0.0
        return max(-abs(limit), min(abs(limit), v))

    def execute_nav_command(self, action: str) -> dict:
        act = (action or "").strip().lower()
        now = time.time()

        if act == "emergency_stop":
            with self._lock:
                self.estop_locked = True
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_estop_fn(True)
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            ok, msg = self._call_service_fn("/waypoint_manager/pause")
            with self._lock:
                self.last_action_message = f"🚨 已触发紧急停止（持续锁定零速；任务暂停: {msg}）"
            return {"ok": True, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "release_emergency_stop":
            with self._lock:
                self.estop_locked = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
                self.last_action_message = "🔓 已解除急停锁定（可点击继续导航或接管手控）"
            self._publish_estop_fn(False)
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            return {"ok": True, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "pause":
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            ok, msg = self._call_service_fn("/waypoint_manager/pause")
            with self._lock:
                self.last_action_message = f"⏸️ 暂停导航：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "resume":
            with self._lock:
                was_estop = self.estop_locked
                self.estop_locked = False
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            if was_estop:
                self._publish_estop_fn(False)
            ok, msg = self._call_service_fn("/waypoint_manager/resume")
            with self._lock:
                self.last_action_message = f"▶️ 继续导航：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "skip":
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            # Arm skip in waypoint_goal_bridge (if running in queue mode), then skip in mission_manager
            self._call_service_fn("/waypoint/goal/skip")
            ok, msg = self._call_service_fn("/waypoint_manager/skip")
            with self._lock:
                self.last_action_message = f"⏭️ 跳过当前目标点：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "cancel":
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            self._call_service_fn("/waypoint/goal/reset")
            ok, msg = self._call_service_fn("/waypoint_manager/cancel")
            with self._lock:
                self.last_action_message = f"⏹️ 取消导航：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        raise ValueError(f"未知的导航控制指令: {action}")

    def set_teleop_takeover(
        self,
        active: bool,
        resume_navigation: bool = False,
        now: Optional[float] = None,
    ) -> dict:
        now = time.time() if now is None else float(now)
        if active:
            with self._lock:
                was_estop = self.estop_locked
                self.estop_locked = False
                self.teleop_active = True
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_vel_at = now
                self.last_action_at = now
            if was_estop:
                self._publish_estop_fn(False)
            self._call_service_fn("/waypoint_manager/pause")
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            with self._lock:
                self.last_action_message = "🕹️ 已接管手控（导航自动暂停，松手自动零速保护）"
        else:
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            if resume_navigation:
                ok, msg = self._call_service_fn("/waypoint_manager/resume")
                with self._lock:
                    self.last_action_message = f"🔄 已释放手控并继续导航（{msg}）"
            else:
                with self._lock:
                    self.last_action_message = "⏸️ 已退出手控模式（导航保持暂停）"
        status = self.get_status()
        return {
            "ok": True,
            "teleop_active": status["teleop_active"],
            "message": status["last_action_message"],
            "control": status,
        }

    def set_teleop_velocity(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        now: Optional[float] = None,
    ) -> dict:
        now = time.time() if now is None else float(now)
        with self._lock:
            if self.estop_locked:
                raise RuntimeError("当前处于急停锁定状态，请先解除急停")
            if not self.teleop_active:
                raise RuntimeError("请先点击「接管控制权」进入手控模式")
            cx = self._clamp(vx, self.max_linear_vel)
            cy = self._clamp(vy, self.max_linear_vel)
            cw = self._clamp(vyaw, self.max_angular_vel)
            self.target_vel = (cx, cy, cw)
            self.last_vel_at = now
        self._publish_cmd_vel_fn(cx, cy, cw)
        return {
            "ok": True,
            "vx": cx,
            "vy": cy,
            "vyaw": cw,
        }

    def tick(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        pub_vel: Optional[Tuple[float, float, float]] = None
        with self._lock:
            if self.estop_locked:
                pub_vel = (0.0, 0.0, 0.0)
            elif self.teleop_active:
                if (now - self.last_vel_at) > self.cmd_timeout_sec:
                    self.target_vel = (0.0, 0.0, 0.0)
                pub_vel = self.target_vel
        if pub_vel is not None:
            self._publish_cmd_vel_fn(pub_vel[0], pub_vel[1], pub_vel[2])

    def get_status(self) -> dict:
        with self._lock:
            return {
                "estop_locked": self.estop_locked,
                "teleop_active": self.teleop_active,
                "target_vel": {
                    "vx": round(self.target_vel[0], 3),
                    "vy": round(self.target_vel[1], 3),
                    "vyaw": round(self.target_vel[2], 3),
                },
                "max_linear_vel": self.max_linear_vel,
                "max_angular_vel": self.max_angular_vel,
                "last_action_message": self.last_action_message,
                "last_action_at": self.last_action_at,
            }


class MonitorStateHub:
    """Central thread-safe state hub shared between ROS 2 callbacks and HTTP server."""

    def __init__(
        self,
        stall_detector: Optional[StallDetector] = None,
        subscribe_image_fn: Optional[Callable[[], None]] = None,
        unsubscribe_image_fn: Optional[Callable[[], None]] = None,
        intercom: Optional[AudioIntercom] = None,
        nav_controller: Optional[NavigationAndTeleopController] = None,
    ):
        self.stall = stall_detector or StallDetector()
        self.camera = OnDemandImageGate(
            subscribe_fn=subscribe_image_fn or (lambda: None),
            unsubscribe_fn=unsubscribe_image_fn or (lambda: None),
            default_fps=2.0,
        )
        self.intercom = intercom or AudioIntercom()
        self.nav = nav_controller or NavigationAndTeleopController()
        self._lock = threading.Lock()
        self.system_health: Optional[dict] = None
        self.started_at = time.time()

    def update_system_health_json(self, raw_json: str) -> None:
        try:
            parsed = json.loads(raw_json)
            with self._lock:
                self.system_health = parsed
        except Exception:
            pass

    def get_snapshot(self) -> dict:
        with self._lock:
            health = self.system_health
        return {
            "ok": True,
            "server_time": time.time(),
            "uptime_sec": round(time.time() - self.started_at, 1),
            "safety": self.stall.evaluate(),
            "camera": self.camera.get_status(),
            "intercom": self.intercom.get_status(),
            "control": self.nav.get_status(),
            "system_health": health,
        }


INDEX_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
<title>ONS2 远程导航监控与喊话台</title>
<style>
  :root {
    --bg: #0f172a;
    --card: #1e293b;
    --border: #334155;
    --text: #f8fafc;
    --muted: #94a3b8;
    --ok: #10b981;
    --warn: #f59e0b;
    --danger: #ef4444;
    --primary: #3b82f6;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg);
    color: var(--text);
    padding: 16px;
    max-width: 1200px;
    margin: 0 auto;
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 12px;
    padding: 14px 18px;
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 12px;
    margin-bottom: 16px;
  }
  .title-group h1 { font-size: 1.25rem; font-weight: 700; }
  .title-group p { font-size: 0.85rem; color: var(--muted); margin-top: 4px; }
  .badge {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 6px 12px;
    border-radius: 999px;
    font-size: 0.85rem;
    font-weight: 600;
  }
  .badge-ok { background: rgba(16,185,129,0.18); color: #34d399; border: 1px solid #059669; }
  .badge-warn { background: rgba(245,158,11,0.18); color: #fbbf24; border: 1px solid #d97706; }
  .badge-danger { background: rgba(239,68,68,0.25); color: #fca5a5; border: 1px solid #dc2626; }

  #alarmBanner {
    display: none;
    padding: 18px 20px;
    border-radius: 12px;
    margin-bottom: 16px;
    background: #dc2626;
    color: #fff;
    border: 2px solid #fca5a5;
    animation: pulseAlarm 0.9s infinite alternate;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 12px;
  }
  @keyframes pulseAlarm {
    from { background: #dc2626; box-shadow: 0 0 12px rgba(239,68,68,0.5); }
    to   { background: #991b1b; box-shadow: 0 0 28px rgba(239,68,68,0.95); }
  }
  .alarm-text h2 { font-size: 1.2rem; margin-bottom: 4px; }
  .alarm-actions { display: flex; gap: 10px; flex-wrap: wrap; }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
    gap: 16px;
  }
  .card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 18px;
  }
  .card h3 {
    font-size: 1.05rem;
    margin-bottom: 14px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 1px solid var(--border);
    padding-bottom: 10px;
  }
  .kv-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    margin-bottom: 14px;
  }
  .kv-box {
    background: rgba(15,23,42,0.6);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 12px;
  }
  .kv-label { font-size: 0.78rem; color: var(--muted); }
  .kv-val { font-size: 1.05rem; font-weight: 700; margin-top: 4px; font-family: monospace; }

  .btn {
    cursor: pointer;
    border: none;
    border-radius: 8px;
    padding: 10px 16px;
    font-size: 0.92rem;
    font-weight: 600;
    color: #fff;
    background: var(--primary);
    transition: opacity 0.15s, transform 0.1s;
  }
  .btn:hover { opacity: 0.92; }
  .btn:active { transform: scale(0.98); }
  .btn-ok { background: var(--ok); }
  .btn-danger { background: var(--danger); }
  .btn-secondary { background: #475569; }
  .btn-outline {
    background: transparent;
    border: 1px solid var(--border);
    color: var(--text);
  }
  .btn-mic {
    width: 100%;
    padding: 16px;
    font-size: 1.05rem;
    background: linear-gradient(135deg, #2563eb, #1d4ed8);
    margin-bottom: 12px;
    user-select: none;
  }
  .btn-mic.recording {
    background: linear-gradient(135deg, #dc2626, #b91c1c);
    animation: pulseAlarm 0.7s infinite alternate;
  }

  .preset-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    margin-top: 10px;
  }
  .preset-btn {
    text-align: left;
    padding: 9px 11px;
    font-size: 0.85rem;
    background: rgba(59,130,246,0.14);
    border: 1px solid rgba(59,130,246,0.4);
    color: #bfdbfe;
    border-radius: 8px;
    cursor: pointer;
  }
  .preset-btn:hover { background: rgba(59,130,246,0.28); }

  .input-row {
    display: flex;
    gap: 8px;
    margin-top: 10px;
  }
  .input-row input[type="text"], .input-row select {
    flex: 1;
    padding: 10px 12px;
    border-radius: 8px;
    border: 1px solid var(--border);
    background: #0f172a;
    color: var(--text);
    font-size: 0.92rem;
  }
  .cam-viewport {
    margin-top: 12px;
    width: 100%;
    min-height: 240px;
    background: #090d16;
    border: 1px dashed var(--border);
    border-radius: 10px;
    display: flex;
    align-items: center;
    justify-content: center;
    overflow: hidden;
    position: relative;
  }
  .cam-viewport img {
    width: 100%;
    height: auto;
    display: none;
  }
  .cam-placeholder {
    color: var(--muted);
    text-align: center;
    padding: 24px;
    font-size: 0.9rem;
    line-height: 1.6;
  }
  .toast {
    margin-top: 10px;
    font-size: 0.85rem;
    color: #93c5fd;
    min-height: 1.2em;
  }
  .nav-btn-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
    gap: 10px;
    margin-bottom: 14px;
  }
  .teleop-box {
    margin-top: 12px;
    padding: 14px;
    border-radius: 10px;
    background: rgba(15, 23, 42, 0.75);
    border: 1px solid var(--border);
  }
  .teleop-box.active {
    border: 2px solid #f59e0b;
    box-shadow: 0 0 16px rgba(245, 158, 11, 0.25);
  }
  .dpad-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 10px;
    max-width: 380px;
    margin: 12px auto 6px auto;
  }
  .dpad-btn {
    padding: 16px 8px;
    font-size: 0.95rem;
    font-weight: 700;
    border-radius: 10px;
    border: 1px solid #475569;
    background: #1e293b;
    color: #f8fafc;
    cursor: pointer;
    user-select: none;
    touch-action: none;
    text-align: center;
  }
  .dpad-btn:active, .dpad-btn.pressed {
    background: #2563eb;
    border-color: #60a5fa;
    transform: scale(0.97);
  }
</style>
</head>
<body>

<header>
  <div class="title-group">
    <h1>🐕 ONS2 远程导航监控与控制台</h1>
    <p>30秒停滞报警 · 急停/暂停/跳点/手控接管 · 远程原声/TTS喊话 · 按需低带宽图像查看</p>
  </div>
  <div style="display:flex; gap:10px; align-items:center; flex-wrap:wrap;">
    <span id="connBadge" class="badge badge-warn">连接中...</span>
    <button class="btn btn-danger" style="padding:10px 18px; font-size:1rem;" onclick="sendNavCommand('emergency_stop')">🛑 紧急停止 (E-STOP)</button>
    <button class="btn btn-secondary" onclick="unlockBrowserAudio()" id="soundUnlockBtn">🔔 激活网页报警音</button>
  </div>
</header>

<div id="estopBanner" style="display:none; padding:16px 20px; border-radius:12px; margin-bottom:16px; background:#7f1d1d; color:#fff; border:2px solid #f87171; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:12px;">
  <div>
    <h2 style="font-size:1.15rem; margin-bottom:4px;">🛑 紧急停止已锁定（正以 25Hz 持续向底盘下发零速）</h2>
    <p style="font-size:0.88rem; color:#fecaca;">导航已暂停，所有运动指令已被拦截。确认现场安全后可解除急停或继续导航。</p>
  </div>
  <div style="display:flex; gap:10px;">
    <button class="btn btn-ok" onclick="sendNavCommand('release_emergency_stop')">🔓 解除急停锁定</button>
    <button class="btn" onclick="sendNavCommand('resume')">▶️ 解除并继续导航</button>
  </div>
</div>

<div id="alarmBanner">
  <div class="alarm-text">
    <h2 id="alarmTitle">🚨 机械狗停滞/原地蠕动警报！</h2>
    <p id="alarmDesc">连续 30 秒净位移低于安全阈值，请立即查看现场画面、手控脱困或远程喊话！</p>
  </div>
  <div class="alarm-actions">
    <button class="btn" style="background:#fff; color:#b91c1c;" onclick="toggleCamera(true)">📷 立即打开画面查看</button>
    <button class="btn btn-ok" onclick="setTeleopTakeover(true)">🕹️ 接管手控脱困</button>
    <button class="btn btn-secondary" onclick="ackAlarm(60)">🔕 静音 60 秒</button>
  </div>
</div>

<div class="grid">
  <!-- 1. 安全监控卡片 -->
  <div class="card">
    <h3>
      <span>🛡️ 1. 安全监控（停滞 / 原地蠕动检测）</span>
      <span id="safetyBadge" class="badge badge-ok">正常</span>
    </h3>
    <div class="kv-grid">
      <div class="kv-box">
        <div class="kv-label">导航任务状态</div>
        <div class="kv-val" id="missionState">--</div>
      </div>
      <div class="kv-box">
        <div class="kv-label">目标点进度 / 距离</div>
        <div class="kv-val" id="wpProgress">--</div>
      </div>
      <div class="kv-box">
        <div class="kv-label">近 30s 净位移半径</div>
        <div class="kv-val" id="maxDisp">0.000 m</div>
      </div>
      <div class="kv-box">
        <div class="kv-label">当前停滞持续时间</div>
        <div class="kv-val" id="stallDur">0.0 s / 30 s</div>
      </div>
      <div class="kv-box">
        <div class="kv-label">当前坐标 (x, y)</div>
        <div class="kv-val" id="odomPos">--</div>
      </div>
      <div class="kv-box">
        <div class="kv-label">实时速度 (vx, wz)</div>
        <div class="kv-val" id="odomVel">--</div>
      </div>
    </div>
    <p id="safetyNote" style="font-size:0.88rem; color:var(--muted); margin-bottom:12px;">等待数据...</p>
    <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
      <button class="btn btn-outline" id="forceMonitorBtn" onclick="toggleForceMonitor()">开启全时段强制监控</button>
      <button class="btn btn-outline" onclick="testAlarmBeep()">🔊 测试网页报警蜂鸣</button>
    </div>
  </div>

  <!-- 2. 远程喊话卡片 -->
  <div class="card">
    <h3>
      <span>📢 2. 远程喊话（麦克风原声 + 文字TTS）</span>
      <span style="font-size:0.8rem; color:var(--muted);" id="playCountLabel">已播报 0 次</span>
    </h3>

    <button class="btn btn-mic" id="micBtn" onclick="toggleMicRecording()">
      🎙️ 点击开始麦克风录音喊话（再点一下立即发送播放）
    </button>

    <div style="margin-bottom:10px; display:flex; align-items:center; justify-content:space-between; font-size:0.83rem; color:var(--muted);">
      <span>或直接录制/上传音频文件发送：</span>
      <input type="file" id="audioFileInput" accept="audio/*" capture="microphone" onchange="uploadAudioFile(this)" style="max-width:190px;" />
    </div>

    <div class="input-row">
      <input type="text" id="ttsInput" placeholder="输入要在机械狗端播报的文字..." onkeydown="if(event.key==='Enter') sendTts()" />
      <button class="btn btn-ok" onclick="sendTts()">🔊 文字播报</button>
    </div>

    <div style="margin-top:10px; display:flex; align-items:center; gap:10px; font-size:0.85rem; color:var(--muted);">
      <span>播报音量：<strong id="volVal">130%</strong></span>
      <input type="range" id="volSlider" min="50" max="220" step="10" value="130" oninput="document.getElementById('volVal').textContent=this.value+'%'" style="flex:1;" />
    </div>

    <div class="preset-grid" id="presetGrid"></div>
    <div class="toast" id="audioToast"></div>
  </div>

  <!-- 3. 导航控制与短暂手控接管卡片 -->
  <div class="card" style="grid-column: 1 / -1;">
    <h3>
      <span>🧭 3. 导航任务控制 & 短暂接管手控</span>
      <span id="controlBadge" class="badge badge-ok">自动导航模式</span>
    </h3>

    <div class="nav-btn-grid">
      <button class="btn btn-danger" onclick="sendNavCommand('emergency_stop')">🛑 紧急停止 (E-Stop)</button>
      <button class="btn btn-secondary" id="releaseEstopBtn" style="display:none;" onclick="sendNavCommand('release_emergency_stop')">🔓 解除急停</button>
      <button class="btn" style="background:#d97706;" onclick="sendNavCommand('pause')">⏸️ 暂停导航</button>
      <button class="btn btn-ok" onclick="sendNavCommand('resume')">▶️ 继续导航</button>
      <button class="btn" style="background:#2563eb;" onclick="sendNavCommand('skip')">⏭️ 跳过此目标点 → 下一目标点</button>
      <button class="btn btn-secondary" onclick="if(confirm('确定取消当前全部导航任务吗？')) sendNavCommand('cancel')">⏹️ 取消导航</button>
    </div>

    <div class="teleop-box" id="teleopBox">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:10px;">
        <div>
          <strong style="font-size:0.98rem;">🕹️ 短暂接管控制权（远程手控脱困）</strong>
          <p style="font-size:0.83rem; color:var(--muted); margin-top:3px;">
            点击「接管控制权」会自动暂停当前导航任务并停止规划器输出；手控调整位置后，可一键释放控制权并继续原导航。
          </p>
        </div>
        <div style="display:flex; gap:8px; flex-wrap:wrap;">
          <button class="btn" id="takeoverBtn" style="background:#f59e0b; color:#0f172a;" onclick="setTeleopTakeover(true)">🕹️ 接管控制权（进入手控）</button>
          <button class="btn btn-ok" id="releaseResumeBtn" style="display:none;" onclick="setTeleopTakeover(false, true)">🔄 释放控制权并继续导航</button>
          <button class="btn btn-secondary" id="releasePauseBtn" style="display:none;" onclick="setTeleopTakeover(false, false)">⏸️ 仅退出手控（保持暂停）</button>
        </div>
      </div>

      <div id="teleopPadArea" style="display:none; margin-top:14px; border-top:1px solid var(--border); padding-top:14px;">
        <div style="display:flex; gap:18px; flex-wrap:wrap; margin-bottom:10px; font-size:0.85rem; color:var(--muted);">
          <label style="flex:1; min-width:200px;">
            手控线速度：<strong id="linSpeedLabel">0.25 m/s</strong>
            <input type="range" id="linSpeedSlider" min="0.10" max="0.60" step="0.05" value="0.25"
              oninput="document.getElementById('linSpeedLabel').textContent=parseFloat(this.value).toFixed(2)+' m/s'" style="width:100%; margin-top:4px;" />
          </label>
          <label style="flex:1; min-width:200px;">
            手控角速度：<strong id="angSpeedLabel">0.40 rad/s</strong>
            <input type="range" id="angSpeedSlider" min="0.15" max="0.80" step="0.05" value="0.40"
              oninput="document.getElementById('angSpeedLabel').textContent=parseFloat(this.value).toFixed(2)+' rad/s'" style="width:100%; margin-top:4px;" />
          </label>
        </div>

        <div class="dpad-grid">
          <button class="dpad-btn" data-dir="turn_left">↶ 左转 (Q)</button>
          <button class="dpad-btn" data-dir="forward">⬆️ 前进 (W)</button>
          <button class="dpad-btn" data-dir="turn_right">↷ 右转 (E)</button>
          <button class="dpad-btn" data-dir="strafe_left">⬅️ 左移 (A)</button>
          <button class="dpad-btn" data-dir="stop" style="background:#7f1d1d; border-color:#ef4444;">⏹️ 停止</button>
          <button class="dpad-btn" data-dir="strafe_right">➡️ 右移 (D)</button>
          <div></div>
          <button class="dpad-btn" data-dir="backward">⬇️ 后退 (S)</button>
          <div></div>
        </div>
        <p style="text-align:center; font-size:0.8rem; color:var(--muted); margin-top:6px;">
          按住按钮或键盘 W/A/S/D/Q/E 持续移动，松开立即自动归零刹停（0.6秒死区保护）
        </p>
      </div>
    </div>

    <div class="toast" id="navControlToast"></div>
  </div>

  <!-- 4. 按需实时图像卡片 -->
  <div class="card" style="grid-column: 1 / -1;">
    <h3>
      <span>📷 4. 现场实时图像（/odin1/image/compressed）</span>
      <span id="camBadge" class="badge badge-ok">已关闭（0 带宽省流中）</span>
    </h3>
    <div style="display:flex; gap:10px; align-items:center; flex-wrap:wrap;">
      <button class="btn btn-ok" id="camToggleBtn" onclick="toggleCamera()">👁️ 开启实时画面</button>
      <label style="font-size:0.88rem; color:var(--muted);">
        传输帧率：
        <select id="camFpsSelect" onchange="changeCameraFps(this.value)" style="background:#0f172a; color:#fff; border:1px solid var(--border); padding:6px 10px; border-radius:6px;">
          <option value="1">1 FPS（极省流）</option>
          <option value="2" selected>2 FPS（推荐 4G）</option>
          <option value="3">3 FPS（上限）</option>
        </select>
      </label>
      <span id="camStats" style="font-size:0.84rem; color:var(--muted); margin-left:auto;">默认不订阅图像话题，不消耗 4G 流量</span>
    </div>

    <div class="cam-viewport">
      <div class="cam-placeholder" id="camPlaceholder">
        当前画面传输已关闭（0 B/s 流量消耗）<br/>
        需要查看机械狗周边情况或收到停滞报警时，点击上方「开启实时画面」即可实时拉流，看完随时关闭。
      </div>
      <img id="camImg" alt="ONS2 Camera Stream" />
    </div>
  </div>
</div>

<script>
let stateData = null;
let audioCtx = null;
let beepTimer = null;
let mediaRecorder = null;
let micChunks = [];
let isRecording = false;
let camPollTimer = null;
let camWatchdogTimer = null;
let camLoopActive = false;
let fetchingStatus = false;
let statusFailCount = 0;
let presetsInitialized = false;

function ensureAudioCtx() {
  if (!audioCtx) {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (AudioContext) audioCtx = new AudioContext();
  }
  if (audioCtx && audioCtx.state === 'suspended') {
    audioCtx.resume();
  }
}

function unlockBrowserAudio() {
  ensureAudioCtx();
  playBeepOnce(880, 0.12);
  if ('Notification' in window && Notification.permission === 'default') {
    Notification.requestPermission();
  }
  const btn = document.getElementById('soundUnlockBtn');
  btn.textContent = '✅ 报警音已就绪';
  btn.className = 'btn btn-ok';
}

function playBeepOnce(freq = 920, duration = 0.22) {
  try {
    ensureAudioCtx();
    if (!audioCtx) return;
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = 'sawtooth';
    osc.frequency.setValueAtTime(freq, audioCtx.currentTime);
    gain.gain.setValueAtTime(0.25, audioCtx.currentTime);
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    osc.start();
    osc.stop(audioCtx.currentTime + duration);
  } catch (e) {}
}

function testAlarmBeep() {
  playBeepOnce(950, 0.22);
  setTimeout(() => playBeepOnce(680, 0.25), 260);
}

function startAlarmLoop() {
  if (beepTimer) return;
  testAlarmBeep();
  beepTimer = setInterval(testAlarmBeep, 1200);
}

function stopAlarmLoop() {
  if (beepTimer) {
    clearInterval(beepTimer);
    beepTimer = null;
  }
}

async function fetchStatus() {
  if (fetchingStatus) return;
  fetchingStatus = true;
  try {
    const ctrl = new AbortController();
    const tid = setTimeout(() => ctrl.abort(), 4000);
    const res = await fetch('/api/status', { cache: 'no-store', signal: ctrl.signal });
    clearTimeout(tid);
    const data = await res.json();
    statusFailCount = 0;
    stateData = data;
    renderStatus(data);
  } catch (e) {
    statusFailCount++;
    if (statusFailCount >= 3) {
      const b = document.getElementById('connBadge');
      b.textContent = '⚠️ 连接波动，重试中...';
      b.className = 'badge badge-danger';
    }
  } finally {
    fetchingStatus = false;
  }
}

function renderStatus(data) {
  const b = document.getElementById('connBadge');
  b.textContent = '🟢 在线连接正常';
  b.className = 'badge badge-ok';

  const s = data.safety;
  document.getElementById('missionState').textContent = s.mission_state || 'UNKNOWN';
  const wpIdx = (s.current_waypoint_idx !== null && s.current_waypoint_idx !== undefined) ? (s.current_waypoint_idx + 1) : '-';
  const wpTot = (s.total_waypoints !== null && s.total_waypoints !== undefined) ? s.total_waypoints : '-';
  const distTxt = (s.distance_to_target_m !== null && s.distance_to_target_m !== undefined && s.distance_to_target_m >= 0)
    ? `${s.distance_to_target_m.toFixed(2)}m` : '--';
  document.getElementById('wpProgress').textContent = `${wpIdx}/${wpTot} (${distTxt})`;
  document.getElementById('maxDisp').textContent = (s.max_displacement_m ?? 0).toFixed(3) + ' m';
  document.getElementById('stallDur').textContent = `${(s.stall_duration_sec ?? 0).toFixed(1)} s / ${s.window_sec} s`;
  document.getElementById('safetyNote').textContent = s.alarm_message || '';

  if (data.control) {
    const c = data.control;
    const estopBanner = document.getElementById('estopBanner');
    const relEstopBtn = document.getElementById('releaseEstopBtn');
    const ctrlBadge = document.getElementById('controlBadge');
    const teleopBox = document.getElementById('teleopBox');
    const takeBtn = document.getElementById('takeoverBtn');
    const relResumeBtn = document.getElementById('releaseResumeBtn');
    const relPauseBtn = document.getElementById('releasePauseBtn');
    const padArea = document.getElementById('teleopPadArea');

    if (c.estop_locked) {
      estopBanner.style.display = 'flex';
      relEstopBtn.style.display = 'inline-block';
      ctrlBadge.textContent = '🛑 急停锁定中';
      ctrlBadge.className = 'badge badge-danger';
    } else {
      estopBanner.style.display = 'none';
      relEstopBtn.style.display = 'none';
      if (c.teleop_active) {
        ctrlBadge.textContent = '🕹️ 远程手控接管中';
        ctrlBadge.className = 'badge badge-warn';
      } else {
        ctrlBadge.textContent = '🧭 自动导航模式';
        ctrlBadge.className = 'badge badge-ok';
      }
    }

    if (c.teleop_active) {
      teleopBox.classList.add('active');
      takeBtn.style.display = 'none';
      relResumeBtn.style.display = 'inline-block';
      relPauseBtn.style.display = 'inline-block';
      padArea.style.display = 'block';
    } else {
      teleopBox.classList.remove('active');
      takeBtn.style.display = 'inline-block';
      relResumeBtn.style.display = 'none';
      relPauseBtn.style.display = 'none';
      padArea.style.display = 'none';
    }
    if (c.last_action_message) {
      document.getElementById('navControlToast').textContent = c.last_action_message;
    }
  }

  if (s.odom) {
    document.getElementById('odomPos').textContent = `${s.odom.x.toFixed(2)}, ${s.odom.y.toFixed(2)}`;
    document.getElementById('odomVel').textContent = `${s.odom.vx.toFixed(2)}m/s, ${s.odom.vyaw.toFixed(2)}r/s`;
  } else {
    document.getElementById('odomPos').textContent = '无数据';
    document.getElementById('odomVel').textContent = '无数据';
  }

  const forceBtn = document.getElementById('forceMonitorBtn');
  forceBtn.textContent = s.force_monitor ? '关闭全时段强制监控' : '开启全时段强制监控';

  const banner = document.getElementById('alarmBanner');
  const sb = document.getElementById('safetyBadge');
  if (s.alarm) {
    banner.style.display = 'flex';
    document.getElementById('alarmDesc').textContent = s.alarm_message;
    sb.textContent = '🚨 停滞报警';
    sb.className = 'badge badge-danger';
    if (!s.buzzer_muted) {
      startAlarmLoop();
    } else {
      stopAlarmLoop();
    }
  } else {
    banner.style.display = 'none';
    sb.textContent = s.monitoring_active ? '🛡️ 监控正常' : '⏸️ 待机';
    sb.className = 'badge badge-ok';
    stopAlarmLoop();
  }

  // Intercom presets
  const ic = data.intercom;
  document.getElementById('playCountLabel').textContent = `已播报 ${ic.play_count} 次` + (ic.last_spoken_summary ? `（最近：${ic.last_spoken_summary}）` : '');
  if (!presetsInitialized && ic.presets) {
    presetsInitialized = true;
    const grid = document.getElementById('presetGrid');
    grid.innerHTML = '';
    ic.presets.forEach(p => {
      const btn = document.createElement('button');
      btn.className = 'preset-btn';
      btn.textContent = '🔊 ' + p;
      btn.onclick = () => sendTts(p);
      grid.appendChild(btn);
    });
  }

  // Camera status
  const cam = data.camera;
  const camBadge = document.getElementById('camBadge');
  const camBtn = document.getElementById('camToggleBtn');
  const camImg = document.getElementById('camImg');
  const camPh = document.getElementById('camPlaceholder');
  if (cam.enabled) {
    camBadge.textContent = `🔴 实时传输中 (${cam.fps} FPS)`;
    camBadge.className = 'badge badge-warn';
    camBtn.textContent = '⏹️ 关闭画面（恢复省流）';
    camBtn.className = 'btn btn-danger';
    document.getElementById('camStats').textContent =
      `已接收 ${cam.total_frames_sent} 帧 · 累计流量 ${cam.total_kb_sent} KB · 当前帧 ${(cam.frame_size_bytes/1024).toFixed(1)} KB`;
    camPh.style.display = 'none';
    camImg.style.display = 'block';
    startCameraFrameLoop(cam.fps);
  } else {
    camBadge.textContent = '已关闭（0 带宽省流中）';
    camBadge.className = 'badge badge-ok';
    camBtn.textContent = '👁️ 开启实时画面';
    camBtn.className = 'btn btn-ok';
    document.getElementById('camStats').textContent = '默认不订阅 /odin1/image/compressed，外网图像流量 0 B/s';
    stopCameraFrameLoop();
    camImg.style.display = 'none';
    camPh.style.display = 'block';
  }
}

function startCameraFrameLoop(fps) {
  if (camLoopActive) return;
  camLoopActive = true;
  const img = document.getElementById('camImg');

  const requestNextFrame = () => {
    if (!camLoopActive || !stateData || !stateData.camera || !stateData.camera.enabled) return;
    if (camWatchdogTimer) clearTimeout(camWatchdogTimer);
    camWatchdogTimer = setTimeout(() => {
      if (camLoopActive) scheduleNext();
    }, 4500);
    img.src = '/api/camera/frame.jpg?t=' + Date.now();
  };

  const scheduleNext = () => {
    if (camWatchdogTimer) {
      clearTimeout(camWatchdogTimer);
      camWatchdogTimer = null;
    }
    if (!camLoopActive || !stateData || !stateData.camera || !stateData.camera.enabled) return;
    const curFps = (stateData && stateData.camera && stateData.camera.fps) ? stateData.camera.fps : (fps || 2);
    const intervalMs = Math.max(330, Math.round(1000 / curFps));
    if (camPollTimer) clearTimeout(camPollTimer);
    camPollTimer = setTimeout(requestNextFrame, intervalMs);
  };

  img.onload = scheduleNext;
  img.onerror = scheduleNext;
  requestNextFrame();
}

function stopCameraFrameLoop() {
  camLoopActive = false;
  if (camPollTimer) {
    clearTimeout(camPollTimer);
    camPollTimer = null;
  }
  if (camWatchdogTimer) {
    clearTimeout(camWatchdogTimer);
    camWatchdogTimer = null;
  }
  const img = document.getElementById('camImg');
  if (img) {
    img.onload = null;
    img.onerror = null;
  }
}

async function toggleCamera(forceState) {
  const cur = stateData && stateData.camera ? stateData.camera.enabled : false;
  const next = (typeof forceState === 'boolean') ? forceState : !cur;
  const fps = parseFloat(document.getElementById('camFpsSelect').value || '2');
  stopCameraFrameLoop();
  await fetch('/api/camera/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: next, fps })
  });
  await fetchStatus();
}

async function changeCameraFps(fps) {
  if (stateData && stateData.camera && stateData.camera.enabled) {
    stopCameraFrameLoop();
    await fetch('/api/camera/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: true, fps: parseFloat(fps) })
    });
    await fetchStatus();
  }
}

async function toggleForceMonitor() {
  const cur = stateData && stateData.safety ? stateData.safety.force_monitor : false;
  await fetch('/api/safety/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ force_monitor: !cur })
  });
  await fetchStatus();
}

async function ackAlarm(muteSec) {
  stopAlarmLoop();
  await fetch('/api/safety/ack', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mute_sec: muteSec })
  });
  await fetchStatus();
}

async function sendTts(presetText) {
  const input = document.getElementById('ttsInput');
  const text = presetText || input.value.trim();
  if (!text) return;
  const volume = parseInt(document.getElementById('volSlider').value || '130', 10);
  const toast = document.getElementById('audioToast');
  toast.textContent = `⏳ 正在机械狗端播报：「${text}」...`;
  try {
    const res = await fetch('/api/audio/speak_tts', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text, volume })
    });
    const out = await res.json();
    if (out.ok) {
      toast.textContent = `✅ 播报完成：「${text}」`;
      if (!presetText) input.value = '';
      fetchStatus();
    } else {
      toast.textContent = `❌ 播报失败：${out.error || '未知错误'}`;
    }
  } catch (e) {
    toast.textContent = `❌ 请求失败：${e.message}`;
  }
}

async function toggleMicRecording() {
  const btn = document.getElementById('micBtn');
  const toast = document.getElementById('audioToast');
  if (isRecording && mediaRecorder) {
    mediaRecorder.stop();
    isRecording = false;
    btn.classList.remove('recording');
    btn.textContent = '🎙️ 点击开始麦克风录音喊话（再点一下立即发送播放）';
    return;
  }

  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    toast.textContent = '⚠️ 当前浏览器通过非 HTTPS 访问，请改用 https:// 链接或使用下方「录制/上传音频文件」按钮';
    return;
  }

  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    micChunks = [];
    mediaRecorder = new MediaRecorder(stream);
    mediaRecorder.ondataavailable = e => { if (e.data && e.data.size > 0) micChunks.push(e.data); };
    mediaRecorder.onstop = async () => {
      stream.getTracks().forEach(t => t.stop());
      const blob = new Blob(micChunks, { type: mediaRecorder.mimeType || 'audio/webm' });
      await sendAudioBlob(blob, mediaRecorder.mimeType || 'audio/webm');
    };
    mediaRecorder.start();
    isRecording = true;
    btn.classList.add('recording');
    btn.textContent = '🔴 正在录音... 点击此处立即发送至机械狗播放';
    toast.textContent = '🎙️ 请对着麦克风说话...';
  } catch (e) {
    toast.textContent = `❌ 无法打开麦克风 (${e.message})，可使用下方文件录制按钮或改用 HTTPS 访问`;
  }
}

async function uploadAudioFile(input) {
  if (!input.files || !input.files[0]) return;
  const file = input.files[0];
  await sendAudioBlob(file, file.name || file.type || 'audio/webm');
  input.value = '';
}

async function sendAudioBlob(blob, mimeInfo) {
  const toast = document.getElementById('audioToast');
  const volume = parseInt(document.getElementById('volSlider').value || '130', 10);
  toast.textContent = `⏳ 正在上传语音 (${(blob.size/1024).toFixed(1)} KB) 并在机械狗端外放...`;
  try {
    const res = await fetch(`/api/audio/speak_raw?volume=${volume}&mime=${encodeURIComponent(mimeInfo)}`, {
      method: 'POST',
      body: blob
    });
    const out = await res.json();
    if (out.ok) {
      toast.textContent = `✅ 原声喊话已在机械狗端播放完毕！`;
      fetchStatus();
    } else {
      toast.textContent = `❌ 播放失败：${out.error || '未知错误'}`;
    }
  } catch (e) {
    toast.textContent = `❌ 上传失败：${e.message}`;
  }
}

async function sendNavCommand(action) {
  const toast = document.getElementById('navControlToast');
  stopTeleopHold();
  toast.textContent = `⏳ 正在执行指令：${action}...`;
  try {
    const res = await fetch('/api/nav/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action })
    });
    const out = await res.json();
    toast.textContent = out.message || (out.ok ? '✅ 指令已执行' : `❌ 失败: ${out.error}`);
    await fetchStatus();
  } catch (e) {
    toast.textContent = `❌ 指令发送失败：${e.message}`;
  }
}

async function setTeleopTakeover(active, resumeNavigation = false) {
  const toast = document.getElementById('navControlToast');
  stopTeleopHold();
  try {
    const res = await fetch('/api/teleop/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ active, resume_navigation: resumeNavigation })
    });
    const out = await res.json();
    toast.textContent = out.message || '';
    await fetchStatus();
  } catch (e) {
    toast.textContent = `❌ 切换手控失败：${e.message}`;
  }
}

let teleopHoldTimer = null;
let currentTeleopDir = null;

function dirToVel(dir) {
  const lin = parseFloat(document.getElementById('linSpeedSlider').value || '0.25');
  const ang = parseFloat(document.getElementById('angSpeedSlider').value || '0.40');
  switch (dir) {
    case 'forward':      return { vx: lin,  vy: 0.0,  vyaw: 0.0 };
    case 'backward':     return { vx: -lin, vy: 0.0,  vyaw: 0.0 };
    case 'strafe_left':  return { vx: 0.0,  vy: lin,  vyaw: 0.0 };
    case 'strafe_right': return { vx: 0.0,  vy: -lin, vyaw: 0.0 };
    case 'turn_left':    return { vx: 0.0,  vy: 0.0,  vyaw: ang };
    case 'turn_right':   return { vx: 0.0,  vy: 0.0,  vyaw: -ang };
    default:             return { vx: 0.0,  vy: 0.0,  vyaw: 0.0 };
  }
}

async function postTeleopVel(vel) {
  try {
    await fetch('/api/teleop/cmd_vel', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(vel)
    });
  } catch (e) {}
}

function startTeleopHold(dir) {
  if (!stateData || !stateData.control || !stateData.control.teleop_active) return;
  if (dir === 'stop') {
    stopTeleopHold();
    return;
  }
  currentTeleopDir = dir;
  postTeleopVel(dirToVel(dir));
  if (teleopHoldTimer) clearInterval(teleopHoldTimer);
  teleopHoldTimer = setInterval(() => {
    if (currentTeleopDir) postTeleopVel(dirToVel(currentTeleopDir));
  }, 180);
}

function stopTeleopHold() {
  currentTeleopDir = null;
  if (teleopHoldTimer) {
    clearInterval(teleopHoldTimer);
    teleopHoldTimer = null;
  }
  if (stateData && stateData.control && stateData.control.teleop_active) {
    postTeleopVel({ vx: 0.0, vy: 0.0, vyaw: 0.0 });
  }
}

document.querySelectorAll('.dpad-btn').forEach(btn => {
  const dir = btn.getAttribute('data-dir');
  const onStart = e => { e.preventDefault(); btn.classList.add('pressed'); startTeleopHold(dir); };
  const onEnd = e => { e.preventDefault(); btn.classList.remove('pressed'); stopTeleopHold(); };
  btn.addEventListener('mousedown', onStart);
  btn.addEventListener('touchstart', onStart, { passive: false });
  btn.addEventListener('mouseup', onEnd);
  btn.addEventListener('mouseleave', onEnd);
  btn.addEventListener('touchend', onEnd);
  btn.addEventListener('touchcancel', onEnd);
});

const keyMap = {
  'w': 'forward', 's': 'backward',
  'a': 'strafe_left', 'd': 'strafe_right',
  'q': 'turn_left', 'e': 'turn_right',
  ' ': 'stop'
};
window.addEventListener('keydown', e => {
  if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
  const dir = keyMap[e.key.toLowerCase()];
  if (dir && currentTeleopDir !== dir) {
    e.preventDefault();
    startTeleopHold(dir);
  }
});
window.addEventListener('keyup', e => {
  if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
  const dir = keyMap[e.key.toLowerCase()];
  if (dir) {
    e.preventDefault();
    stopTeleopHold();
  }
});
window.addEventListener('blur', stopTeleopHold);

setInterval(fetchStatus, 1000);
fetchStatus();
</script>
</body>
</html>
"""


class DualProtocolHTTPServer(ThreadingHTTPServer):
    """Single-port HTTP + HTTPS server that performs TLS peek & handshake in worker threads."""

    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 64

    def __init__(self, server_address, RequestHandlerClass, ssl_context: Optional[ssl.SSLContext] = None):
        super().__init__(server_address, RequestHandlerClass)
        self.ssl_context = ssl_context

    def process_request_thread(self, request, client_address):
        if self.ssl_context is not None:
            try:
                request.settimeout(5.0)
                first_byte = request.recv(1, socket.MSG_PEEK)
                if not first_byte:
                    self.shutdown_request(request)
                    return
                if first_byte == b"\x16":
                    request = self.ssl_context.wrap_socket(request, server_side=True)
                request.settimeout(60.0)
            except Exception:
                try:
                    self.shutdown_request(request)
                except Exception:
                    pass
                return
        super().process_request_thread(request, client_address)


def ensure_self_signed_cert(cert_dir: Optional[str] = None) -> Optional[ssl.SSLContext]:
    """Generates a self-signed certificate so the same port serves both HTTP and HTTPS."""
    cert_dir = cert_dir or os.path.join(tempfile.gettempdir(), "ons2_monitor_tls")
    os.makedirs(cert_dir, exist_ok=True)
    cert_path = os.path.join(cert_dir, "cert.pem")
    key_path = os.path.join(cert_dir, "key.pem")
    if not (os.path.isfile(cert_path) and os.path.isfile(key_path)):
        if not shutil.which("openssl"):
            return None
        try:
            subprocess.run(
                [
                    "openssl", "req", "-x509", "-newkey", "rsa:2048",
                    "-keyout", key_path, "-out", cert_path,
                    "-days", "3650", "-nodes",
                    "-subj", "/CN=ons2-remote-monitor",
                ],
                check=True,
                capture_output=True,
                timeout=10,
            )
        except Exception:
            return None
    try:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(certfile=cert_path, keyfile=key_path)
        return ctx
    except Exception:
        return None


def make_handler_class(hub: MonitorStateHub):
    class MonitorRequestHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):
            pass

        def handle(self):
            try:
                super().handle()
            except (ConnectionResetError, BrokenPipeError, TimeoutError, ssl.SSLError, OSError):
                pass

        def _send_json(self, payload: dict, status: int = 200):
            raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _read_body(self) -> bytes:
            length = int(self.headers.get("Content-Length", "0") or "0")
            if length <= 0:
                return b""
            return self.rfile.read(min(length, 15 * 1024 * 1024))

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path

            if path in ("/", "/index.html"):
                raw = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
                return

            if path == "/api/status" or path == "/health":
                self._send_json(hub.get_snapshot())
                return

            if path == "/api/camera/frame.jpg":
                jpeg, _, _ = hub.camera.get_latest_frame()
                if not jpeg and hub.camera.enabled:
                    jpeg, _, _ = hub.camera.wait_for_next_frame(0, timeout=0.8)
                if not jpeg:
                    self.send_response(204)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
                return

            self._send_json({"ok": False, "error": "Not Found"}, status=404)

        def do_POST(self):
            parsed = urlparse(self.path)
            path = parsed.path
            qs = parse_qs(parsed.query)
            body = self._read_body()

            try:
                if path == "/api/camera/control":
                    req = json.loads(body.decode("utf-8") or "{}")
                    enabled = bool(req.get("enabled", False))
                    fps = req.get("fps")
                    cam_status = hub.camera.set_enabled(enabled, fps=fps)
                    self._send_json({"ok": True, "camera": cam_status})
                    return

                if path == "/api/safety/control":
                    req = json.loads(body.decode("utf-8") or "{}")
                    if "force_monitor" in req:
                        hub.stall.set_force_monitor(bool(req["force_monitor"]))
                    if "window_sec" in req or "stall_radius_m" in req:
                        hub.stall.set_config(
                            window_sec=req.get("window_sec"),
                            stall_radius_m=req.get("stall_radius_m"),
                        )
                    self._send_json({"ok": True, "safety": hub.stall.evaluate()})
                    return

                if path == "/api/safety/ack":
                    req = json.loads(body.decode("utf-8") or "{}")
                    mute_sec = float(req.get("mute_sec", 60.0))
                    hub.stall.acknowledge_alarm(mute_sec=mute_sec)
                    self._send_json({"ok": True, "safety": hub.stall.evaluate()})
                    return

                if path == "/api/audio/speak_tts":
                    req = json.loads(body.decode("utf-8") or "{}")
                    res = hub.intercom.speak_tts(
                        text=req.get("text", ""),
                        volume=int(req.get("volume", 100)),
                        voice=req.get("voice"),
                    )
                    self._send_json(res)
                    return

                if path == "/api/audio/speak_raw":
                    volume = int((qs.get("volume") or ["100"])[0])
                    mime = (qs.get("mime") or [self.headers.get("Content-Type", "webm")])[0]
                    res = hub.intercom.speak_raw_audio(body, mime_or_ext=mime, volume=volume)
                    self._send_json(res)
                    return

                if path == "/api/nav/command":
                    req = json.loads(body.decode("utf-8") or "{}")
                    res = hub.nav.execute_nav_command(req.get("action", ""))
                    self._send_json(res)
                    return

                if path == "/api/teleop/control":
                    req = json.loads(body.decode("utf-8") or "{}")
                    res = hub.nav.set_teleop_takeover(
                        active=bool(req.get("active", False)),
                        resume_navigation=bool(req.get("resume_navigation", False)),
                    )
                    self._send_json(res)
                    return

                if path == "/api/teleop/cmd_vel":
                    req = json.loads(body.decode("utf-8") or "{}")
                    res = hub.nav.set_teleop_velocity(
                        vx=float(req.get("vx", 0.0)),
                        vy=float(req.get("vy", 0.0)),
                        vyaw=float(req.get("vyaw", 0.0)),
                    )
                    self._send_json(res)
                    return

                self._send_json({"ok": False, "error": "Not Found"}, status=404)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, status=400)

    return MonitorRequestHandler


def create_monitor_http_server(
    host: str,
    port: int,
    hub: MonitorStateHub,
    enable_tls: bool = True,
) -> Tuple[DualProtocolHTTPServer, threading.Thread, int]:
    ssl_ctx = ensure_self_signed_cert() if enable_tls else None
    handler_cls = make_handler_class(hub)
    server = DualProtocolHTTPServer((host, port), handler_cls, ssl_context=ssl_ctx)
    actual_port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread, actual_port


def run_ros2_monitor_node(args: argparse.Namespace) -> None:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, Empty, String
    from std_srvs.srv import Trigger

    try:
        from nav_interfaces.msg import MissionStatus
        has_mission_status = True
    except ImportError:
        MissionStatus = None
        has_mission_status = False

    class Sw01RemoteMonitorNode(Node):
        def __init__(self, hub: MonitorStateHub):
            super().__init__("remote_monitor")
            self.hub = hub
            self._img_sub = None
            self._sub_lock = threading.Lock()
            self._cb_group = ReentrantCallbackGroup()

            self._cmd_vel_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
            self._neupan_stop_pub = self.create_publisher(Empty, "/neupan/stop", 10)
            self._estop_pub = self.create_publisher(Bool, "/emergency_stop", 10)

            self._srv_clients = {}
            for srv_name in (
                "/waypoint_manager/pause",
                "/waypoint_manager/resume",
                "/waypoint_manager/cancel",
                "/waypoint_manager/skip",
                "/waypoint/goal/skip",
                "/waypoint/goal/reset",
            ):
                self._srv_clients[srv_name] = self.create_client(
                    Trigger, srv_name, callback_group=self._cb_group
                )

            self.create_subscription(
                Odometry, args.odom_topic, self._odom_cb, qos_profile_sensor_data
            )
            self.create_subscription(
                Twist, args.cmd_vel_topic, self._cmd_cb, qos_profile_sensor_data
            )
            self.create_subscription(
                String, "/waypoint_manager/system_health", self._health_cb, qos_profile_sensor_data
            )
            if has_mission_status:
                self.create_subscription(
                    MissionStatus, "/waypoint_manager/status", self._status_cb, qos_profile_sensor_data
                )

            # 25Hz timer for E-Stop zero-velocity lock & Manual Teleop /cmd_vel output
            self.create_timer(0.04, self._control_tick, callback_group=self._cb_group)

        def _control_tick(self) -> None:
            self.hub.nav.tick()

        def publish_cmd_vel(self, vx: float, vy: float, vyaw: float) -> None:
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(vyaw)
            self._cmd_vel_pub.publish(msg)

        def publish_neupan_stop(self) -> None:
            self._neupan_stop_pub.publish(Empty())

        def publish_estop(self, active: bool) -> None:
            msg = Bool()
            msg.data = bool(active)
            self._estop_pub.publish(msg)

        def call_trigger_service(self, srv_name: str, timeout_sec: float = 2.0) -> Tuple[bool, str]:
            client = self._srv_clients.get(srv_name)
            if client is None:
                return False, f"未知服务 {srv_name}"
            if not client.wait_for_service(timeout_sec=0.2):
                return False, f"服务 {srv_name} 当前未就绪（导航栈可能未启动）"
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                if future.done():
                    try:
                        res = future.result()
                        return bool(res.success), str(res.message)
                    except Exception as exc:
                        return False, str(exc)
                time.sleep(0.02)
            return False, f"调用服务 {srv_name} 超时"

        def subscribe_image(self) -> None:
            with self._sub_lock:
                if self._img_sub is None:
                    self._img_sub = self.create_subscription(
                        CompressedImage,
                        args.image_topic,
                        self._image_cb,
                        qos_profile_sensor_data,
                    )
                    self.get_logger().info(f"Camera subscription ENABLED on {args.image_topic}")

        def unsubscribe_image(self) -> None:
            with self._sub_lock:
                if self._img_sub is not None:
                    self.destroy_subscription(self._img_sub)
                    self._img_sub = None
                    self.get_logger().info(f"Camera subscription DISABLED on {args.image_topic} (0 bandwidth)")

        def _odom_cb(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            self.hub.stall.update_odom(p.x, p.y, vx=v.x, vy=v.y, vyaw=w.z, yaw=yaw)

        def _cmd_cb(self, msg: Twist) -> None:
            self.hub.stall.update_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

        def _status_cb(self, msg) -> None:
            self.hub.stall.update_mission_state(
                msg.state,
                distance_to_target_m=msg.distance_to_target_m,
                progress_percent=msg.progress_percent,
                current_waypoint_idx=getattr(msg, "current_waypoint_idx", None),
                total_waypoints=getattr(msg, "total_waypoints", None),
            )

        def _health_cb(self, msg: String) -> None:
            self.hub.update_system_health_json(msg.data)

        def _image_cb(self, msg: CompressedImage) -> None:
            self.hub.camera.on_compressed_frame(bytes(msg.data))

    rclpy.init()
    stall = StallDetector(
        window_sec=args.stall_window_sec,
        stall_radius_m=args.stall_radius_m,
    )
    hub = MonitorStateHub(stall_detector=stall)
    node = Sw01RemoteMonitorNode(hub)
    hub.camera._subscribe_fn = node.subscribe_image
    hub.camera._unsubscribe_fn = node.unsubscribe_image
    hub.nav._call_service_fn = node.call_trigger_service
    hub.nav._publish_cmd_vel_fn = node.publish_cmd_vel
    hub.nav._publish_neupan_stop_fn = node.publish_neupan_stop
    hub.nav._publish_estop_fn = node.publish_estop

    server, _, actual_port = create_monitor_http_server(args.host, args.port, hub, enable_tls=True)
    node.get_logger().info(
        f"ONS2 Remote Web Monitor listening on http://{args.host}:{actual_port} "
        f"and https://{args.host}:{actual_port} (single-port HTTP/HTTPS)"
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ONS2 Remote Web Monitor & Intercom")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=18088, help="Bind port (default: 18088)")
    parser.add_argument("--odom-topic", default="/odin1/odometry", help="Odometry topic")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="cmd_vel topic")
    parser.add_argument("--image-topic", default="/odin1/image/compressed", help="CompressedImage topic")
    parser.add_argument("--stall-window-sec", type=float, default=30.0, help="Stall/creep window in seconds")
    parser.add_argument("--stall-radius-m", type=float, default=0.20, help="Min net displacement radius in meters")
    return parser.parse_args()


if __name__ == "__main__":
    run_ros2_monitor_node(parse_args())
