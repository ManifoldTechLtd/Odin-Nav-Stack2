/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Pure-logic periodic-replan install gate (ROS-independent).
//
// Decision pipeline (evaluated in order):
//   1) Hard reject: candidate flagged with REQUIRES_ALIGNMENT or severe initial
//      heading degradation (exempted when the active path is blocked).
//   2) Evidence gate: candidate is only installed if the active path is blocked
//      (after temporal hysteresis) or the candidate differs meaningfully from
//      the active path; otherwise returns REJECT_NO_CHANGE.
//   3) Side lock: when not blocked, rejects candidates that flip lateral bypass
//      side relative to the active path unless they offer a significant
//      improvement in length, kink angle, or clearance (REJECT_SIDE_LOCK).

#ifndef MISSION_MANAGER_REPLAN_INSTALL_GATE_H
#define MISSION_MANAGER_REPLAN_INSTALL_GATE_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace mission_manager_gate {

// Basic 2D geometry helpers

struct Pt {
  double x{0.0};
  double y{0.0};
};

inline double normDeg180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

struct NearestResult {
  double dist{std::numeric_limits<double>::infinity()};
  double arc{0.0};      // Arc length along path (m); < 0 if projection precedes start
  int seg{0};           // Matched segment index
  bool ok{false};
};

// Closest point on a polyline: returns distance and arc length (arc < 0 if before start).
inline NearestResult nearestOnPolyline(const std::vector<Pt> &path, const Pt &p) {
  NearestResult out;
  if (path.size() < 2) return out;
  double arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Pt &a = path[i];
    const Pt &b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double seg = std::hypot(dx, dy);
    if (seg < 1e-9) continue;
    const double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (seg * seg);
    double tc = t;
    bool behind_start = false;
    if (t < 0.0) {
      tc = 0.0;
      if (i == 0) behind_start = true;
    } else if (t > 1.0) {
      tc = 1.0;
    }
    const Pt q{a.x + tc * dx, a.y + tc * dy};
    const double d = std::hypot(p.x - q.x, p.y - q.y);
    if (d < out.dist) {
      out.dist = d;
      out.arc = behind_start ? -1.0 : (arc + tc * seg);
      out.seg = static_cast<int>(i);
      out.ok = true;
    }
    arc += seg;
  }
  return out;
}

// Interpolate point at arc length s along the path (clamps to endpoints if out of range).
inline bool pointAtArc(const std::vector<Pt> &path, double s, Pt &out) {
  if (path.size() < 2) return false;
  if (s <= 0.0) {
    out = path.front();
    return true;
  }
  double arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Pt &a = path[i];
    const Pt &b = path[i + 1];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < 1e-9) continue;
    if (arc + seg >= s) {
      const double r = (s - arc) / seg;
      out.x = a.x + r * (b.x - a.x);
      out.y = a.y + r * (b.y - a.y);
      return true;
    }
    arc += seg;
  }
  out = path.back();
  return false;
}

inline double pathLength(const std::vector<Pt> &path) {
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

// Maximum kink angle (degrees) between consecutive segments (ignores segments < 0.05m).
inline double maxKinkDeg(const std::vector<Pt> &path) {
  double worst = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const Pt &a = path[i - 1];
    const Pt &b = path[i];
    const Pt &c = path[i + 1];
    const double l1 = std::hypot(b.x - a.x, b.y - a.y);
    const double l2 = std::hypot(c.x - b.x, c.y - b.y);
    if (l1 < 0.05 || l2 < 0.05) continue;
    const double h1 = std::atan2(b.y - a.y, b.x - a.x);
    const double h2 = std::atan2(c.y - b.y, c.x - b.x);
    const double turn = std::fabs(normDeg180(rad2deg(h2 - h1)));
    worst = std::max(worst, turn);
  }
  return worst;
}

// Bidirectional maximum deviation (meters) between two polylines sampled every 0.25m.
inline double maxDeviation(const std::vector<Pt> &a, const std::vector<Pt> &b) {
  if (a.size() < 2 || b.size() < 2) return 0.0;
  double worst = 0.0;
  const double step = 0.25;
  const double len = pathLength(a);
  for (double s = 0.0; s <= len; s += step) {
    Pt p;
    if (!pointAtArc(a, s, p)) break;
    const NearestResult n = nearestOnPolyline(b, p);
    if (n.ok) worst = std::max(worst, n.dist);
  }
  const double lenb = pathLength(b);
  for (double s = 0.0; s <= lenb; s += step) {
    Pt p;
    if (!pointAtArc(b, s, p)) break;
    const NearestResult n = nearestOnPolyline(a, p);
    if (n.ok) worst = std::max(worst, n.dist);
  }
  return worst;
}

// Lateral side sign at arc_m relative to robot pose and yaw:
// +1 (left) if lateral > deadband_m, -1 (right) if lateral < -deadband_m, else 0.
inline double lateralAtArc(const std::vector<Pt> &path, const Pt &robot,
                           double yaw_rad, double arc_m, double deadband_m,
                           double *lateral_out = nullptr) {
  Pt p;
  if (!pointAtArc(path, arc_m, p)) {
    if (path.empty()) return 0.0;
    p = path.back();
  }
  const double fx = std::cos(yaw_rad);
  const double fy = std::sin(yaw_rad);
  const double lateral = (fx * (p.y - robot.y)) - (fy * (p.x - robot.x));
  if (lateral_out) *lateral_out = lateral;
  if (lateral > deadband_m) return 1.0;
  if (lateral < -deadband_m) return -1.0;
  return 0.0;
}

// Signed initial heading demand (degrees) at lookahead_m relative to robot yaw.
inline double startHeadingDeg(const std::vector<Pt> &path, const Pt &robot,
                              double yaw_rad, double lookahead_m) {
  Pt p;
  if (!pointAtArc(path, lookahead_m, p)) {
    if (path.size() < 2) return 0.0;
    p = path.back();
  }
  const double heading = std::atan2(p.y - robot.y, p.x - robot.x);
  return normDeg180(rad2deg(heading - yaw_rad));
}

// Trim polyline starting from the point closest to the robot up to forward_m ahead.
inline std::vector<Pt> trimFromNearest(const std::vector<Pt> &path,
                                       const Pt &robot, double forward_m) {
  std::vector<Pt> out;
  if (path.size() < 2) return out;
  const NearestResult n = nearestOnPolyline(path, robot);
  if (!n.ok) return out;
  const double start_arc = std::max(0.0, n.arc);
  Pt p0;
  pointAtArc(path, start_arc, p0);
  out.push_back(p0);
  double arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Pt &a = path[i];
    const Pt &b = path[i + 1];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < 1e-9) continue;
    if (arc + seg < start_arc) {
      arc += seg;
      continue;
    }
    const double r = std::max(0.0, (start_arc - arc) / seg);
    const Pt entry{a.x + r * (b.x - a.x), a.y + r * (b.y - a.y)};
    if (out.back().x != entry.x || out.back().y != entry.y) out.push_back(entry);
    if (arc + seg - start_arc >= forward_m) {
      const double need = start_arc + forward_m - arc;
      const double rr = need / seg;
      Pt end{a.x + rr * (b.x - a.x), a.y + rr * (b.y - a.y)};
      if (std::hypot(end.x - out.back().x, end.y - out.back().y) > 1e-6) {
        out.push_back(end);
      }
      return out;
    }
    out.push_back(b);
    arc += seg;
  }
  return out;
}

// Corridor occupancy check

struct CorridorHit {
  bool blocked{false};
  double min_lateral_m{-1.0};
  int hits{0};
};

// Check whether any scan point in scan_pts (map frame) falls within half_width_m
// of the forward-trimmed path polyline.
inline CorridorHit corridorBlocked(const std::vector<Pt> &trimmed,
                                   const std::vector<Pt> &scan_pts,
                                   double half_width_m) {
  CorridorHit out;
  if (trimmed.size() < 2 || scan_pts.empty()) return out;
  for (const Pt &p : scan_pts) {
    const NearestResult n = nearestOnPolyline(trimmed, p);
    if (!n.ok) continue;
    if (n.arc < 0.0) continue;
    if (out.min_lateral_m < 0.0 || n.dist < out.min_lateral_m) {
      out.min_lateral_m = n.dist;
    }
    if (n.dist < half_width_m) {
      out.hits += 1;
      out.blocked = true;
    }
  }
  return out;
}

// Saturating time-based hysteresis for the blocked state

class BlockedHysteresis {
 public:
  BlockedHysteresis(double enter_sec, double exit_sec)
      : enter_sec_(std::max(0.0, enter_sec)), exit_sec_(std::max(0.0, exit_sec)) {}

  bool update(double now_sec, bool raw_blocked) {
    if (raw_blocked) {
      clear_since_ = -1.0;
      if (blocked_since_ < 0.0) blocked_since_ = now_sec;
      if (!gated_ && (now_sec - blocked_since_) >= enter_sec_) gated_ = true;
      if (enter_sec_ <= 0.0) gated_ = true;
    } else {
      blocked_since_ = -1.0;
      if (!gated_) return false;
      if (exit_sec_ <= 0.0) {
        gated_ = false;
        clear_since_ = -1.0;
        return gated_;
      }
      if (clear_since_ < 0.0) clear_since_ = now_sec;
      if ((now_sec - clear_since_) >= exit_sec_) {
        gated_ = false;
        clear_since_ = -1.0;
      }
    }
    return gated_;
  }

  void reset() {
    gated_ = false;
    blocked_since_ = -1.0;
    clear_since_ = -1.0;
  }

  bool gated() const { return gated_; }
  bool raw() const { return blocked_since_ >= 0.0; }
  double dwellSec(double now_sec) const {
    return blocked_since_ < 0.0 ? 0.0 : std::max(0.0, now_sec - blocked_since_);
  }

 private:
  double enter_sec_{0.15};
  double exit_sec_{0.20};
  bool gated_{false};
  double blocked_since_{-1.0};
  double clear_since_{-1.0};
};

// Install gate configuration and evaluation

struct GateConfig {
  // Corridor blocked check
  double blocked_half_width_m{0.225};
  double blocked_forward_m{4.0};
  int scan_stride{3};
  double blocked_enter_sec{0.15};
  double blocked_exit_sec{0.20};
  // Geometry comparison window
  double metric_window_m{10.0};
  double heading_lookahead_m{1.0};
  double side_arc_m{2.5};
  double side_deadband_m{0.15};
  // Layer 1: Hard reject
  bool reject_requires_alignment{true};
  bool alignment_reject_skip_when_blocked{true};
  double heading_worsen_deg{25.0};
  bool heading_reject_skip_when_blocked{true};
  // Layer 2: Meaningful change thresholds
  double changed_lateral_m{0.20};
  double changed_length_ratio{1.15};
  double changed_kink_deg{10.0};
  // Layer 3: Significant improvement thresholds (allow side flip)
  double improve_length_ratio{0.90};
  double improve_kink_deg{15.0};
  double improve_clearance_m{0.15};
};

struct PathMetrics {
  bool valid{false};
  double length_m{0.0};
  double max_kink_deg{0.0};
  double start_heading_deg{0.0};
  double side{0.0};             // +1 left / -1 right / 0 straight
  double side_lateral_m{0.0};
  double min_scan_lateral_m{-1.0};
};

struct GateInput {
  PathMetrics active;
  PathMetrics candidate;
  bool candidate_requires_alignment{false};
  bool blocked_gated{false};
  bool blocked_raw{false};
  double blocked_dwell_sec{0.0};
  double deviation_m{0.0};
};

struct GateDecision {
  bool install{true};
  std::string reason;  // OK_INSTALL / REJECT_*
  std::string detail;
};

inline PathMetrics computePathMetrics(const std::vector<Pt> &path,
                                      const Pt &robot, double yaw_rad,
                                      const GateConfig &cfg) {
  PathMetrics m;
  if (path.size() < 2) return m;
  m.valid = true;
  m.length_m = pathLength(path);
  m.max_kink_deg = maxKinkDeg(path);
  m.start_heading_deg = startHeadingDeg(path, robot, yaw_rad, cfg.heading_lookahead_m);
  m.side = lateralAtArc(path, robot, yaw_rad, cfg.side_arc_m, cfg.side_deadband_m,
                        &m.side_lateral_m);
  return m;
}

inline bool anyImprovement(const PathMetrics &a, const PathMetrics &c,
                           const GateConfig &cfg) {
  if (!a.valid) return true;
  if (c.length_m <= a.length_m * cfg.improve_length_ratio) return true;
  if (a.max_kink_deg - c.max_kink_deg >= cfg.improve_kink_deg) return true;
  if (a.min_scan_lateral_m >= 0.0 && c.min_scan_lateral_m >= 0.0 &&
      c.min_scan_lateral_m >= a.min_scan_lateral_m + cfg.improve_clearance_m) {
    return true;
  }
  return false;
}

inline bool meaningfulChange(const PathMetrics &a, const PathMetrics &c,
                             const GateInput &in, const GateConfig &cfg) {
  if (!a.valid) return true;
  if (in.deviation_m >= cfg.changed_lateral_m) return true;
  if (a.length_m > 1e-6 && c.length_m >= a.length_m * cfg.changed_length_ratio) {
    return true;
  }
  if (c.max_kink_deg - a.max_kink_deg >= cfg.changed_kink_deg) return true;
  return false;
}

inline GateDecision evaluateInstallGate(const GateInput &in,
                                        const GateConfig &cfg) {
  GateDecision d;
  char buf[320];

  // Layer 1a: Reject candidate requiring pre-alignment (unless exempted while blocked)
  if (cfg.reject_requires_alignment && in.candidate_requires_alignment &&
      !(cfg.alignment_reject_skip_when_blocked && in.blocked_gated)) {
    d.install = false;
    d.reason = "REJECT_REQUIRES_ALIGNMENT";
    d.detail = "candidate flagged REQUIRES_ALIGNMENT";
    return d;
  }

  // Layer 1b: Reject candidate with significantly worse initial heading demand
  const bool heading_worse =
      std::fabs(in.candidate.start_heading_deg) >
      std::fabs(in.active.start_heading_deg) + cfg.heading_worsen_deg;
  if (heading_worse &&
      !(cfg.heading_reject_skip_when_blocked && in.blocked_gated)) {
    d.install = false;
    d.reason = "REJECT_HEADING_WORSEN";
    std::snprintf(buf, sizeof(buf), "heading act=%.1fdeg cand=%.1fdeg (+%.1f)",
                  in.active.start_heading_deg, in.candidate.start_heading_deg,
                  cfg.heading_worsen_deg);
    d.detail = buf;
    return d;
  }

  const bool improved = anyImprovement(in.active, in.candidate, cfg);
  const bool changed = meaningfulChange(in.active, in.candidate, in, cfg);
  const bool side_conflict = in.active.side != 0.0 && in.candidate.side != 0.0 &&
                             in.active.side != in.candidate.side;

  // Layer 2: Keep active path if not blocked and candidate shows no meaningful change
  if (!in.blocked_gated && !changed && !improved) {
    d.install = false;
    d.reason = "REJECT_NO_CHANGE";
    std::snprintf(buf, sizeof(buf),
                  "dev=%.2fm len act=%.2f cand=%.2f kink act=%.1f cand=%.1f",
                  in.deviation_m, in.active.length_m, in.candidate.length_m,
                  in.active.max_kink_deg, in.candidate.max_kink_deg);
    d.detail = buf;
    return d;
  }

  // Layer 3: Side lock -- reject opposite-side candidate unless blocked or significantly better
  if (!in.blocked_gated && side_conflict && !improved) {
    d.install = false;
    d.reason = "REJECT_SIDE_LOCK";
    std::snprintf(buf, sizeof(buf),
                  "side act=%.0f cand=%.0f lat act=%.2fm cand=%.2fm",
                  in.active.side, in.candidate.side, in.active.side_lateral_m,
                  in.candidate.side_lateral_m);
    d.detail = buf;
    return d;
  }

  d.install = true;
  d.reason = "OK_INSTALL";
  std::snprintf(buf, sizeof(buf),
                "blocked=%d(raw=%d dwell=%.2fs) side act=%.0f cand=%.0f "
                "dev=%.2fm improved=%d",
                in.blocked_gated ? 1 : 0, in.blocked_raw ? 1 : 0,
                in.blocked_dwell_sec, in.active.side, in.candidate.side,
                in.deviation_m, improved ? 1 : 0);
  d.detail = buf;
  return d;
}

}  // namespace mission_manager_gate

#endif  // MISSION_MANAGER_REPLAN_INSTALL_GATE_H
