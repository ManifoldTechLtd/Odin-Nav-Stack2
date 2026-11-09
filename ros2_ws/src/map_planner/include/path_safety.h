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
#include <limits>

namespace map_planner_safety {

// Coordinates are in cells. Check every crossed cell, including both sides of
// boundaries and all four cells at a corner; point sampling can miss these.
template <class FreeCell>
bool segmentFree(double x0, double y0, double x1, double y1, FreeCell free_cell) {
  const double bound = static_cast<double>(std::numeric_limits<int>::max()) - 2.0;
  for (double v : {x0, y0, x1, y1}) {
    if (!std::isfinite(v) || std::abs(v) > bound) return false;
  }
  if (std::abs(x1 - x0) + std::abs(y1 - y0) > 1000000.0) return false;
  auto check = [&](double x, double y) {
    const bool xb = std::abs(x - std::round(x)) < 1e-9;
    const bool yb = std::abs(y - std::round(y)) < 1e-9;
    const int xhi = static_cast<int>(xb ? std::round(x) : std::floor(x));
    const int yhi = static_cast<int>(yb ? std::round(y) : std::floor(y));
    for (int ix = xhi - (xb ? 1 : 0); ix <= xhi; ++ix) {
      for (int iy = yhi - (yb ? 1 : 0); iy <= yhi; ++iy) {
        if (!free_cell(ix, iy)) return false;
      }
    }
    return true;
  };
  if (!check(x0, y0)) return false;
  const double dx = x1 - x0, dy = y1 - y0;
  const double infinity = std::numeric_limits<double>::infinity();
  const int sx = dx > 0.0 ? 1 : -1, sy = dy > 0.0 ? 1 : -1;
  double bx = dx > 0.0 ? std::floor(x0) + 1.0 : std::ceil(x0) - 1.0;
  double by = dy > 0.0 ? std::floor(y0) + 1.0 : std::ceil(y0) - 1.0;
  double tx = std::abs(dx) < 1e-12 ? infinity : (bx - x0) / dx;
  double ty = std::abs(dy) < 1e-12 ? infinity : (by - y0) / dy;
  double previous = 0.0;
  // Merge the ordered X/Y boundary crossings on the fly. Recompute crossing
  // times from integer boundaries to avoid accumulated drift on long rays.
  while (true) {
    const double t = std::min(1.0, std::min(tx, ty));
    const double mid = 0.5 * (previous + t);
    if (!check(x0 + dx * mid, y0 + dy * mid) ||
        !check(x0 + dx * t, y0 + dy * t)) return false;
    if (t >= 1.0) break;
    if (tx <= t) {
      bx += sx;
      tx = (bx - x0) / dx;
    }
    if (ty <= t) {
      by += sy;
      ty = (by - y0) / dy;
    }
    previous = t;
  }
  return true;
}

}  // namespace map_planner_safety
