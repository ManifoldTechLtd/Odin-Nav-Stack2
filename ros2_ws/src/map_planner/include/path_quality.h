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

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace map_planner_quality {

struct Point2d {
  double x{0.0};
  double y{0.0};
};

struct GeometryMetrics {
  double heading_error_deg{0.0};
  double max_kink_deg{0.0};
};

inline double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double headingErrorDeg(double start_yaw, double dx, double dy) {
  if (std::hypot(dx, dy) <= 1e-9) return 0.0;
  return std::fabs(wrapAngle(std::atan2(dy, dx) - start_yaw)) * 180.0 / M_PI;
}

inline double turnPenaltyCost(double previous_dx, double previous_dy,
                              double next_dx, double next_dy, double weight) {
  const double previous_norm = std::hypot(previous_dx, previous_dy);
  const double next_norm = std::hypot(next_dx, next_dy);
  if (weight <= 0.0 || previous_norm <= 1e-9 || next_norm <= 1e-9) return 0.0;
  const double cosine = std::max(-1.0, std::min(
      1.0, (previous_dx * next_dx + previous_dy * next_dy) /
               (previous_norm * next_norm)));
  return weight * (1.0 - cosine);
}

inline GeometryMetrics computeGeometryMetrics(
    const std::vector<Point2d>& points, double start_yaw,
    double heading_arc_m = 1.0, double kink_sample_m = 0.3) {
  GeometryMetrics metrics;
  if (points.size() < 2) return metrics;

  std::vector<double> arc(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    arc[i] = arc[i - 1] +
             std::hypot(points[i].x - points[i - 1].x,
                        points[i].y - points[i - 1].y);
  }
  const double total = arc.back();
  if (total <= 1e-9) return metrics;

  auto pointAtArc = [&](double target) {
    target = std::max(0.0, std::min(target, total));
    auto it = std::lower_bound(arc.begin(), arc.end(), target);
    if (it == arc.begin()) return points.front();
    if (it == arc.end()) return points.back();
    const std::size_t hi = static_cast<std::size_t>(it - arc.begin());
    const std::size_t lo = hi - 1;
    const double span = arc[hi] - arc[lo];
    if (span <= 1e-9) return points[hi];
    const double u = (target - arc[lo]) / span;
    return Point2d{points[lo].x + u * (points[hi].x - points[lo].x),
                   points[lo].y + u * (points[hi].y - points[lo].y)};
  };

  const Point2d heading_point = pointAtArc(std::min(heading_arc_m, total));
  metrics.heading_error_deg = headingErrorDeg(
      start_yaw, heading_point.x - points.front().x,
      heading_point.y - points.front().y);

  const double sample_step = std::max(kink_sample_m, 1e-3);
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Point2d before = pointAtArc(std::max(0.0, arc[i] - sample_step));
    const Point2d after = pointAtArc(std::min(total, arc[i] + sample_step));
    const double dx1 = points[i].x - before.x;
    const double dy1 = points[i].y - before.y;
    const double dx2 = after.x - points[i].x;
    const double dy2 = after.y - points[i].y;
    if (std::hypot(dx1, dy1) <= 1e-9 || std::hypot(dx2, dy2) <= 1e-9) {
      continue;
    }
    const double kink = std::fabs(
        wrapAngle(std::atan2(dy2, dx2) - std::atan2(dy1, dx1))) *
        180.0 / M_PI;
    metrics.max_kink_deg = std::max(metrics.max_kink_deg, kink);
  }
  return metrics;
}

}
