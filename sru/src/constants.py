"""Constants and configuration values for the SRU navigation controller.

Defaults are conservative; override at runtime through the companion
YAML / launch parameters.
"""

# ---------------------------------------------------------------------------
# Control parameters
# ---------------------------------------------------------------------------
DEFAULT_CONTROL_FREQUENCY = 5.0       # Hz (matches training)
DEFAULT_MIN_DEPTH = 0.25               # meters
DEFAULT_MAX_DEPTH = 10.0               # meters
# Depth freshness: control loop is timer-driven at control_frequency using the
# latest depth frame. If the newest frame is older than this, the node publishes
# an explicit zero twist instead of acting on stale perception (and instead of
# going silent, which would let the chassis bridge hit its 0.5 s timeout
# and inject zero-velocity jerks).
DEPTH_STALE_SEC = 0.5                  # seconds
ARRIVE_GOAL_THRESHOLD = 0.6            # meters (aligned with training in_goal=0.5m + margin)

# ---------------------------------------------------------------------------
# Policy output scaling: [linear_x, linear_y, angular_z].
# Conservative defaults; raise only after the robot is verified to track
# commands safely. Override at runtime through ~policy_scale.
# ---------------------------------------------------------------------------
POLICY_SCALE = [0.6, 0.3, 0.6]
LATERAL_VELOCITY_SCALE = 0.6           # extra damping on linear_y

# Low-pass filter coefficients (alpha): output = a*new + (1-a)*prev
LOW_PASS_FILTER_COEF = [0.9, 0.5, 0.5]

# ---------------------------------------------------------------------------
# Timer intervals
# ---------------------------------------------------------------------------
TARGET_VECTOR_PUBLISH_INTERVAL = 0.2   # 5 Hz

# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
TWIST_MARKER_SCALE = 5.0
TWIST_MARKER_ID = 0
TARGET_VECTOR_MARKER_ID = 1

# ---------------------------------------------------------------------------
# Physics
# ---------------------------------------------------------------------------
GRAVITY_MAGNITUDE = 9.81               # m/s^2