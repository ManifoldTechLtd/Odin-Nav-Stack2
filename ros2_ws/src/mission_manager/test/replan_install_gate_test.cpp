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

// Unit tests for replan_install_gate.h (standalone, no ROS or gtest dependency).

#include "mission_manager/replan_install_gate.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace mission_manager_gate;

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (cond) {                                                              \
      ++g_pass;                                                              \
    } else {                                                                 \
      ++g_fail;                                                              \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
    }                                                                        \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
  do {                                                                       \
    const double va = (a);                                                   \
    const double vb = (b);                                                   \
    if (std::fabs(va - vb) <= (tol)) {                                       \
      ++g_pass;                                                              \
    } else {                                                                 \
      ++g_fail;                                                              \
      std::printf("FAIL %s:%d  |%.6f - %.6f| > %.6f\n", __FILE__, __LINE__,  \
                  va, vb, static_cast<double>(tol));                         \
    }                                                                        \
  } while (0)

static std::vector<Pt> straight(double len, double step = 0.25) {
  std::vector<Pt> p;
  for (double s = 0.0; s <= len + 1e-9; s += step) p.push_back(Pt{s, 0.0});
  return p;
}

int main() {
  // Basic geometry
  {
    const std::vector<Pt> path = straight(5.0);
    const NearestResult n = nearestOnPolyline(path, Pt{2.0, 0.3});
    CHECK(n.ok);
    CHECK_NEAR(n.dist, 0.3, 1e-6);
    CHECK_NEAR(n.arc, 2.0, 1e-6);

    const NearestResult behind = nearestOnPolyline(path, Pt{-0.5, 0.1});
    CHECK(behind.ok);
    CHECK(behind.arc < 0.0);

    CHECK_NEAR(pathLength(path), 5.0, 1e-6);
    CHECK_NEAR(maxKinkDeg(path), 0.0, 1e-9);
  }
  {
    std::vector<Pt> path = straight(2.0, 0.5);
    for (double s = 0.5; s <= 3.0 + 1e-9; s += 0.5) path.push_back(Pt{2.0, s});
    CHECK_NEAR(maxKinkDeg(path), 90.0, 1e-6);
  }
  {
    std::vector<Pt> path;
    for (double s = 0.0; s <= 4.0 + 1e-9; s += 0.25) path.push_back(Pt{s, 0.5});
    const Pt robot{0.0, 0.0};
    double lateral = 0.0;
    const double side = lateralAtArc(path, robot, 0.0, 2.5, 0.15, &lateral);
    CHECK_NEAR(side, 1.0, 1e-9);
    CHECK_NEAR(lateral, 0.5, 1e-6);

    const std::vector<Pt> st = straight(5.0);
    CHECK_NEAR(lateralAtArc(st, robot, 0.0, 2.5, 0.15), 0.0, 1e-9);

    std::vector<Pt> diag;
    for (double s = 0.0; s <= 3.0; s += 0.25) diag.push_back(Pt{s, s});
    CHECK_NEAR(startHeadingDeg(diag, robot, 0.0, 1.0), 45.0, 1e-6);
  }
  {
    const std::vector<Pt> path = straight(10.0);
    const std::vector<Pt> t = trimFromNearest(path, Pt{1.0, 0.0}, 2.0);
    CHECK(t.size() >= 2);
    CHECK_NEAR(pathLength(t), 2.0, 1e-6);
    CHECK_NEAR(t.front().x, 1.0, 1e-6);
    CHECK_NEAR(t.back().x, 3.0, 1e-6);
  }
  {
    const std::vector<Pt> a = straight(5.0);
    std::vector<Pt> b;
    for (double s = 0.0; s <= 5.0 + 1e-9; s += 0.25) b.push_back(Pt{s, 0.5});
    CHECK_NEAR(maxDeviation(a, a), 0.0, 1e-9);
    CHECK_NEAR(maxDeviation(a, b), 0.5, 1e-6);
  }

  // Corridor occupancy
  {
    const GateConfig cfg;
    const std::vector<Pt> trimmed = straight(cfg.blocked_forward_m);
    std::vector<Pt> scan;
    scan.push_back(Pt{1.5, 0.10});
    scan.push_back(Pt{3.5, 1.20});
    const CorridorHit h = corridorBlocked(trimmed, scan, cfg.blocked_half_width_m);
    CHECK(h.blocked);
    CHECK_NEAR(h.min_lateral_m, 0.10, 1e-6);
    CHECK(h.hits == 1);
  }
  {
    const GateConfig cfg;
    const std::vector<Pt> trimmed = straight(cfg.blocked_forward_m);
    std::vector<Pt> scan;
    scan.push_back(Pt{1.5, 0.60});
    scan.push_back(Pt{3.0, -0.80});
    const CorridorHit h = corridorBlocked(trimmed, scan, cfg.blocked_half_width_m);
    CHECK(!h.blocked);
    CHECK_NEAR(h.min_lateral_m, 0.60, 1e-6);
  }
  {
    const GateConfig cfg;
    const std::vector<Pt> trimmed = straight(cfg.blocked_forward_m);
    std::vector<Pt> scan{Pt{-1.0, 0.05}};
    const CorridorHit h = corridorBlocked(trimmed, scan, cfg.blocked_half_width_m);
    CHECK(!h.blocked);
  }

  // Time-based hysteresis
  {
    BlockedHysteresis h(0.15, 0.20);
    CHECK(!h.update(0.00, true));
    CHECK(!h.update(0.10, true));
    CHECK(!h.update(0.12, false));
    CHECK(!h.gated());
  }
  {
    BlockedHysteresis h(0.15, 0.20);
    CHECK(!h.update(0.00, true));
    CHECK(!h.update(0.10, true));
    CHECK(h.update(0.16, true));
    CHECK(h.gated());
    CHECK(h.update(0.24, false));
    CHECK(h.gated());
    CHECK(h.update(0.40, false));
    CHECK(h.gated());
    CHECK(!h.update(0.45, false));
    CHECK(!h.gated());
  }
  {
    BlockedHysteresis h(0.15, 0.20);
    double t = 0.0;
    bool ever = false;
    for (int i = 0; i < 20; ++i) {
      ever = ever || h.update(t, true);
      t += 0.08;
      ever = ever || h.update(t, false);
      t += 0.08;
    }
    CHECK(!ever);
  }
  {
    BlockedHysteresis h(0.15, 0.20);
    h.update(0.00, true);
    h.update(0.20, true);
    CHECK(h.gated());
    h.update(0.30, false);
    h.update(0.38, true);
    CHECK(h.gated());
    CHECK_NEAR(h.dwellSec(0.38), 0.0, 1e-9);
  }

  // Install gate decision chain
  const Pt robot{0.0, 0.0};
  const double yaw = 0.0;
  GateConfig cfg;

  auto makeSide = [](double lateral) {
    std::vector<Pt> p;
    for (double s = 0.0; s <= 5.0 + 1e-9; s += 0.25) {
      p.push_back(Pt{s, lateral * (s / 5.0)});
    }
    return p;
  };

  {
    // 1) REQUIRES_ALIGNMENT -> rejected
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate_requires_alignment = true;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(!d.install);
    CHECK(d.reason == "REJECT_REQUIRES_ALIGNMENT");
  }
  {
    // 2) Not blocked + no meaningful change -> REJECT_NO_CHANGE
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.deviation_m = 0.02;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(!d.install);
    CHECK(d.reason == "REJECT_NO_CHANGE");
  }
  {
    // 3) Not blocked + opposite side + no improvement -> REJECT_SIDE_LOCK
    GateInput in;
    in.active = computePathMetrics(makeSide(1.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(makeSide(-1.0), robot, yaw, cfg);
    in.deviation_m = 1.5;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(in.active.side == 1.0);
    CHECK(in.candidate.side == -1.0);
    CHECK(!d.install);
    CHECK(d.reason == "REJECT_SIDE_LOCK");
  }
  {
    // 4) Opposite side with significant length improvement -> allowed
    PathMetrics a = computePathMetrics(makeSide(1.0), robot, yaw, cfg);
    PathMetrics c = computePathMetrics(makeSide(-1.0), robot, yaw, cfg);
    a.length_m = 6.0;
    c.length_m = 5.0;
    GateInput in;
    in.active = a;
    in.candidate = c;
    in.deviation_m = 1.5;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(d.install);
    CHECK(d.reason == "OK_INSTALL");
  }
  {
    // 5) Blocked -> allowed
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.blocked_gated = true;
    in.blocked_raw = true;
    in.blocked_dwell_sec = 1.2;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(d.install);
    CHECK(d.reason == "OK_INSTALL");
  }
  {
    // 6) Not blocked + 40 deg heading degradation -> REJECT_HEADING_WORSEN
    std::vector<Pt> rotated;
    for (double s = 0.0; s <= 5.0 + 1e-9; s += 0.25) {
      const double a = deg2rad(40.0);
      rotated.push_back(Pt{s * std::cos(a), s * std::sin(a)});
    }
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(rotated, robot, yaw, cfg);
    in.deviation_m = 2.0;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(!d.install);
    CHECK(d.reason == "REJECT_HEADING_WORSEN");
  }
  {
    // 7) Blocked + heading degradation -> exempted
    std::vector<Pt> rotated;
    for (double s = 0.0; s <= 5.0 + 1e-9; s += 0.25) {
      const double a = deg2rad(40.0);
      rotated.push_back(Pt{s * std::cos(a), s * std::sin(a)});
    }
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(rotated, robot, yaw, cfg);
    in.deviation_m = 2.0;
    in.blocked_gated = true;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(d.install);
  }
  {
    // 8) No active path -> allowed
    GateInput in;
    in.candidate = computePathMetrics(straight(5.0), robot, yaw, cfg);
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(d.install);
  }
  {
    // 9) REQUIRES_ALIGNMENT + blocked -> exempted
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate = computePathMetrics(straight(5.0), robot, yaw, cfg);
    in.candidate_requires_alignment = true;
    in.blocked_gated = true;
    const GateDecision d = evaluateInstallGate(in, cfg);
    CHECK(d.install);
    CHECK(d.reason == "OK_INSTALL");
  }
  {
    // 10) REQUIRES_ALIGNMENT + blocked with exemption disabled -> rejected
    GateConfig cfg2 = cfg;
    cfg2.alignment_reject_skip_when_blocked = false;
    GateInput in;
    in.active = computePathMetrics(straight(5.0), robot, yaw, cfg2);
    in.candidate = computePathMetrics(straight(5.0), robot, yaw, cfg2);
    in.candidate_requires_alignment = true;
    in.blocked_gated = true;
    const GateDecision d = evaluateInstallGate(in, cfg2);
    CHECK(!d.install);
    CHECK(d.reason == "REJECT_REQUIRES_ALIGNMENT");
  }

  std::printf("replan_install_gate_test: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
