"""On-demand camera gate: subscribe to frames only while a client is watching."""

import threading
import time
from typing import Callable, Optional, Tuple


MAX_FPS = 15.0


class OnDemandImageGate:
    """Controls dynamic ROS 2 subscription to /odin1/image/compressed with fast JPEG downscaling."""

    def __init__(
        self,
        subscribe_fn: Callable[[], None],
        unsubscribe_fn: Callable[[], None],
        default_fps: float = 8.0,
        max_width: int = 420,
        jpeg_quality: int = 35,
        idle_timeout_sec: float = 300.0,
    ):
        self._subscribe_fn = subscribe_fn
        self._unsubscribe_fn = unsubscribe_fn
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self.enabled = False
        self.fps = max(1.0, min(float(default_fps), MAX_FPS))
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
        if len(raw_jpeg) <= 8 * 1024 or self.max_width <= 0:
            return raw_jpeg
        try:
            import cv2
            import numpy as np

            buf = np.frombuffer(raw_jpeg, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is not None:
                h, w = img.shape[:2]
                if w > self.max_width:
                    new_h = max(1, int(round(h * (self.max_width / float(w)))))
                    img = cv2.resize(img, (self.max_width, new_h), interpolation=cv2.INTER_AREA)
                ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if ok:
                    return enc.tobytes()
        except Exception:
            pass
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
                im.save(out, format="JPEG", quality=self.jpeg_quality)
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
                self.fps = max(1.0, min(float(fps), MAX_FPS))
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
            self.last_client_pull_at = time.time()
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


