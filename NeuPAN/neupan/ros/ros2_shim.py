# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 1 (rospy / tf) to ROS 2 Jazzy compatibility shim for neupan_ros.py.

Design notes:
1. Preserves neupan_ros.py control logic without modification by providing compatible
   `rospy` and `tf` namespaces.
2. `rospy.Time` subclasses `builtin_interfaces.msg.Time` and patches `to_nsec()` /
   `to_sec()` onto `builtin_interfaces.msg.Time` for incoming message timestamps.
3. Node binding:
   - Embedded mode: host rclpy node calls `bind_node(self)` prior to instantiating
     `neupan_core`, sharing the host node's executor.
   - Standalone mode: lazily creates a node and background `MultiThreadedExecutor`.
4. QoS: publishers and subscribers default to RELIABLE with `depth = max(queue_size, 10)`.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import rclpy
import rclpy.executors
import rclpy.qos
import tf2_ros
from builtin_interfaces.msg import Time as _BuiltinTime

_node = None                       # type: ignore[var-annotated]
_owns_node = False
_executor = None                   # type: ignore[var-annotated]
_executor_thread = None            # type: ignore[var-annotated]
_sub_keepalive: list = []
_pub_keepalive: list = []


def bind_node(node) -> None:
    """Attach the shim to a host rclpy node before constructing neupan_core."""
    global _node
    _node = node


def _ensure_node(name: str = "neupan_node", anonymous: bool = False):
    """Return the active rclpy node, creating and spinning one in standalone mode if needed."""
    global _node, _owns_node, _executor, _executor_thread
    if _node is not None:
        return _node
    node_name = name
    if anonymous:
        node_name = "%s_%d" % (name, int(time.time() * 1000) % 100000)
    _node = rclpy.create_node(node_name)
    _owns_node = True
    _executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    _executor.add_node(_node)
    _executor_thread = threading.Thread(target=_executor.spin, daemon=True)
    _executor_thread.start()
    return _node


class _Time(_BuiltinTime):  # type: ignore[misc]
    """ROS 1-compatible Time subclassing builtin_interfaces.msg.Time."""

    def __init__(self, sec: float = 0, nsec: float = 0):  # noqa: D102
        super().__init__()
        self.sec = int(sec)
        self.nanosec = int(nsec)

    @classmethod
    def now(cls) -> "_Time":
        node = _ensure_node()
        ns = node.get_clock().now().nanoseconds
        return cls(ns // 10**9, ns % 10**9)

    @classmethod
    def from_msg(cls, msg) -> "_Time":
        return cls(msg.sec, msg.nanosec)

    def to_sec(self) -> float:
        return float(self.sec) + float(self.nanosec) * 1e-9

    def to_nsec(self) -> int:
        return int(self.sec) * 10**9 + int(self.nanosec)

    def is_zero(self) -> bool:
        return self.sec == 0 and self.nanosec == 0

    def __repr__(self) -> str:  # pragma: no cover
        return "rospy.Time(sec=%d, nsec=%d)" % (self.sec, self.nanosec)


if not hasattr(_BuiltinTime, "to_nsec"):
    def _builtin_to_nsec(self):
        return int(self.sec) * 10**9 + int(self.nanosec)

    def _builtin_to_sec(self):
        return float(self.sec) + float(self.nanosec) * 1e-9

    _BuiltinTime.to_nsec = _builtin_to_nsec  # type: ignore[attr-defined]
    _BuiltinTime.to_sec = _builtin_to_sec    # type: ignore[attr-defined]


class _Duration:
    """Minimal ROS 1 Duration implementation."""

    def __init__(self, sec: float = 0):
        self._sec = float(sec)

    def sleep(self) -> None:
        time.sleep(max(self._sec, 0.0))

    def to_sec(self) -> float:
        return self._sec


class _Rate:
    def __init__(self, hz: float):
        self._period = 1.0 / max(float(hz), 1e-6)
        self._next = time.monotonic() + self._period
        self._lock = threading.Lock()

    def sleep(self) -> None:
        with self._lock:
            now = time.monotonic()
            delay = self._next - now
            if delay > 0:
                time.sleep(delay)
            self._next = max(self._next + self._period, time.monotonic())


def _depth_of(queue_size) -> int:
    try:
        qs = int(queue_size)
    except (TypeError, ValueError):
        qs = 0
    return max(qs, 10)


import array as _array_mod

_NATIVE_SCALARS = (float, int, bool, str, bytes, type(None))
_array_types = (_array_mod.array,)
_FIELD_TYPE_CACHE: dict = {}


def _field_types_of(cls) -> dict:
    """Return cached {field_name: idl_type_name} mapping for a ROS 2 message class."""
    ft = _FIELD_TYPE_CACHE.get(cls)
    if ft is None:
        try:
            ft = cls.get_fields_and_field_types()
        except Exception:  # noqa: BLE001
            ft = {}
        _FIELD_TYPE_CACHE[cls] = ft
    return ft


def _coerce(val, ftype: str):
    """Coerce a scalar value to the exact native Python type expected by rclpy C extensions."""
    try:
        if ftype in ("double", "float"):
            return val if type(val) is float else float(val)
        if ftype.startswith("int") or ftype.startswith("uint"):
            return val if type(val) is int else int(val)
        if ftype == "bool":
            return val if type(val) is bool else bool(val)
    except Exception:  # noqa: BLE001
        return val
    return val


def _safe_set(obj, name, value) -> None:
    try:
        setattr(obj, name, value)
    except Exception:  # noqa: BLE001
        pass


def _sanitize_msg(obj, _depth: int = 0) -> None:
    """Sanitize a message tree in-place before rclpy C serialization:
    1. Convert `_Time` instances to native `builtin_interfaces.msg.Time`.
    2. Convert numpy scalars/arrays to native Python scalars/lists.
    3. Coerce numeric scalars to match IDL field types (`float` vs `int`).
    """
    if obj is None or _depth > 8:
        return
    ftypes = _field_types_of(type(obj))
    for slot in (getattr(type(obj), "__slots__", ()) or ()):
        pub = slot.lstrip("_")
        if not pub or not hasattr(type(obj), pub):
            continue
        try:
            val = getattr(obj, pub)
        except Exception:  # noqa: BLE001
            continue
        if isinstance(val, _array_types):
            continue
        ftype = ftypes.get(pub, "")
        if isinstance(val, _Time):
            _safe_set(obj, pub, _BuiltinTime(sec=int(val.sec), nanosec=int(val.nanosec)))
            continue
        if hasattr(val, "tolist"):
            try:
                native = val.tolist()
            except Exception:  # noqa: BLE001
                continue
            _safe_set(obj, pub, native if isinstance(native, list) else _coerce(native, ftype))
            continue
        if type(val) in _NATIVE_SCALARS:
            new = _coerce(val, ftype)
            if new is not val and type(new) is not type(val):
                _safe_set(obj, pub, new)
            continue
        if isinstance(val, (list, tuple)):
            for i, item in enumerate(val):
                if hasattr(item, "tolist"):
                    try:
                        val[i] = item.tolist()
                    except Exception:  # noqa: BLE001
                        pass
                else:
                    _sanitize_msg(item, _depth + 1)
        elif hasattr(type(val), "__slots__"):
            _sanitize_msg(val, _depth + 1)


class _Publisher:
    def __init__(self, topic: str, msg_type, queue_size: int = 10):
        node = _ensure_node()
        self._pub = node.create_publisher(msg_type, topic, _depth_of(queue_size))
        _pub_keepalive.append(self)

    def publish(self, msg) -> None:
        _sanitize_msg(msg)
        self._pub.publish(msg)


class _Subscriber:
    def __init__(self, topic: str, msg_type, callback: Callable, queue_size=None):
        node = _ensure_node()
        qos = rclpy.qos.QoSProfile(
            depth=_depth_of(queue_size),
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._sub = node.create_subscription(msg_type, topic, callback, qos)
        _sub_keepalive.append(self)


_declared_params: set = set()


def get_param(name: str, default: Any = None) -> Any:
    node = _ensure_node()
    key = name.lstrip("/")
    try:
        if not node.has_parameter(key):
            node.declare_parameter(key, default)
    except Exception:
        pass
    try:
        value = node.get_parameter(key).value
    except Exception:
        value = None
    return default if value is None else value


def init_node(name: str = "neupan_node", anonymous: bool = False, **kwargs) -> None:
    node = _ensure_node(name, anonymous=anonymous)
    node.get_logger().debug("[ros2_shim] init_node(%s) -> node=%s" % (name, node.get_name()))


def is_shutdown() -> bool:
    return not rclpy.ok()


def get_rostime() -> _Time:
    return _Time.now()


class ROSInterruptException(Exception):
    pass


_throttle_last: dict = {}
_once_seen: set = set()
_log_lock = threading.Lock()


def _fmt_msg(fmt: str, args) -> str:
    if not args:
        return fmt
    try:
        return fmt % args
    except Exception:
        return " ".join([fmt] + [str(a) for a in args])


def _log(level: str, fmt: str, args) -> None:
    node = _ensure_node()
    logger = node.get_logger()
    text = _fmt_msg(fmt, args)
    # Keep each severity call on a distinct source line because rclpy caches logger
    # severity per (file, line) caller ID.
    try:
        if level == "debug":
            logger.debug(text)
        elif level == "warn":
            logger.warning(text)
        elif level == "error":
            logger.error(text)
        elif level == "fatal":
            logger.fatal(text)
        else:
            logger.info(text)
    except Exception:  # noqa: BLE001
        print(f"[neupan:{level}] {text}", flush=True)


def _log_throttle(level: str, period: float, fmt: str, args) -> None:
    key = (level, fmt)
    now = time.monotonic()
    with _log_lock:
        last = _throttle_last.get(key)
        if last is not None and (now - last) < float(period):
            return
        _throttle_last[key] = now
    _log(level, fmt, args)


def _log_once(level: str, fmt: str, args) -> None:
    key = (level, fmt)
    with _log_lock:
        if key in _once_seen:
            return
        _once_seen.add(key)
    _log(level, fmt, args)


class _TFException(Exception):
    pass


class LookupException(_TFException):
    pass


class ConnectivityException(_TFException):
    pass


class ExtrapolationException(_TFException):
    pass


class TransformListener:
    def __init__(self, *args, **kwargs):
        node = _ensure_node()
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(
            self._buffer, node,
            qos=rclpy.qos.QoSProfile(
                depth=100, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self._last_error = None
        _log(
            "info",
            "[shim] TransformListener attached: passed=%s actual_node=%s",
            (
                (getattr(node, "get_name", lambda: "?")() if node is not None else "None"),
                (getattr(getattr(self._listener, "node", None), "get_name", lambda: "?")()
                 if getattr(self._listener, "node", None) is not None else "None"),
            ),
        )

    def lookupTransform(self, target_frame: str, source_frame: str, time_point, timeout=None):
        """ROS 1-compatible lookupTransform returning ((x, y, z), (qx, qy, qz, qw))."""
        try:
            zero = isinstance(time_point, _Time) and time_point.is_zero()
            if zero:
                lookup_time = rclpy.time.Time()
            else:
                lookup_time = rclpy.time.Time(
                    seconds=int(getattr(time_point, "sec", 0)),
                    nanoseconds=int(getattr(time_point, "nanosec", 0)),
                )
            timeout_obj = rclpy.duration.Duration()
            if timeout is not None:
                sec = timeout.to_sec() if hasattr(timeout, "to_sec") else float(timeout)
                timeout_obj = rclpy.duration.Duration(seconds=sec)
            ts = self._buffer.lookup_transform(
                target_frame, source_frame, lookup_time, timeout_obj
            )
            stamp_ns = ts.header.stamp.sec * 10**9 + ts.header.stamp.nanosec
            # Static-only chains have stamp zero. Historical lookups retain
            # exact-time semantics; live control may not use stale latest poses.
            if zero and stamp_ns:
                age = (_ensure_node().get_clock().now().nanoseconds - stamp_ns) * 1e-9
                if age < -0.1 or age > 1.0:
                    raise ValueError("latest TF is not fresh (age %.3fs, limit 1.0s)" % age)
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)
            try:
                _log_throttle(
                    "info", 5.0, "[shim] tf lookup failed (%s <- %s): %s",
                    (target_frame, source_frame, exc),
                )
            except Exception:  # noqa: BLE001
                pass
            raise LookupException(str(exc)) from exc

        t = ts.transform.translation
        r = ts.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)


class _RosPyNamespace:
    init_node = staticmethod(init_node)
    Publisher = _Publisher
    Subscriber = _Subscriber
    Rate = _Rate
    Time = _Time
    Duration = _Duration
    get_rostime = staticmethod(get_rostime)
    get_param = staticmethod(get_param)
    is_shutdown = staticmethod(is_shutdown)
    ROSInterruptException = ROSInterruptException

    @staticmethod
    def loginfo(fmt, *args):
        _log("info", fmt, args)

    @staticmethod
    def logwarn(fmt, *args):
        _log("warn", fmt, args)

    @staticmethod
    def logerr(fmt, *args):
        _log("error", fmt, args)

    @staticmethod
    def loginfo_throttle(period, fmt, *args):
        _log_throttle("info", period, fmt, args)

    @staticmethod
    def logwarn_throttle(period, fmt, *args):
        _log_throttle("warn", period, fmt, args)

    @staticmethod
    def logerr_throttle(period, fmt, *args):
        _log_throttle("error", period, fmt, args)

    @staticmethod
    def loginfo_once(fmt, *args):
        _log_once("info", fmt, args)

    @staticmethod
    def logwarn_once(fmt, *args):
        _log_once("warn", fmt, args)


class _TfNamespace:
    TransformListener = TransformListener
    LookupException = LookupException
    ConnectivityException = ConnectivityException
    ExtrapolationException = ExtrapolationException


rospy = _RosPyNamespace()
tf = _TfNamespace()
