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

#include <gtest/gtest.h>

#include <limits>
#include <cstdlib>
#include <new>
#include <set>
#include <utility>
#include <vector>

#include "path_safety.h"

namespace {
bool track_allocations = false;
std::size_t allocation_count = 0;

template <class FreeCell>
bool previousConservativeTraversal(double x0, double y0, double x1, double y1,
                                   FreeCell free_cell) {
  auto check = [&](double x, double y) {
    const bool xb = std::abs(x - std::round(x)) < 1e-9;
    const bool yb = std::abs(y - std::round(y)) < 1e-9;
    const int xhi = static_cast<int>(xb ? std::round(x) : std::floor(x));
    const int yhi = static_cast<int>(yb ? std::round(y) : std::floor(y));
    for (int ix = xhi - (xb ? 1 : 0); ix <= xhi; ++ix)
      for (int iy = yhi - (yb ? 1 : 0); iy <= yhi; ++iy)
        if (!free_cell(ix, iy)) return false;
    return true;
  };
  std::vector<double> crossings{0.0, 1.0};
  auto add = [&](double a, double b) {
    if (std::abs(b - a) < 1e-12) return;
    const int last = static_cast<int>(std::floor(std::max(a, b)));
    for (int k = static_cast<int>(std::floor(std::min(a, b))) + 1; k <= last; ++k) {
      const double t = (k - a) / (b - a);
      if (t > 0.0 && t < 1.0) crossings.push_back(t);
    }
  };
  add(x0, x1);
  add(y0, y1);
  std::sort(crossings.begin(), crossings.end());
  for (std::size_t i = 0; i < crossings.size(); ++i) {
    const double t = crossings[i];
    if (!check(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)) return false;
    if (i > 0) {
      const double mid = 0.5 * (crossings[i - 1] + t);
      if (!check(x0 + (x1 - x0) * mid, y0 + (y1 - y0) * mid)) return false;
    }
  }
  return true;
}
}  // namespace

void* operator new(std::size_t size) {
  if (track_allocations) ++allocation_count;
  if (void* p = std::malloc(size ? size : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

TEST(PathSafety, SparseSegmentCannotJumpOverOccupiedCell) {
  auto free = [](int x, int y) { return x != 4 || y != 2; };
  EXPECT_FALSE(map_planner_safety::segmentFree(1.5, 2.5, 8.5, 2.5, free));
  EXPECT_TRUE(map_planner_safety::segmentFree(1.5, 3.5, 8.5, 3.5, free));
}

TEST(PathSafety, CornerTouchingCellsAreCheckedInBothDirections) {
  auto free = [](int x, int y) { return x != 2 || y != 1; };
  EXPECT_FALSE(map_planner_safety::segmentFree(1.5, 1.5, 3.5, 3.5, free));
  EXPECT_FALSE(map_planner_safety::segmentFree(3.5, 3.5, 1.5, 1.5, free));
}

TEST(PathSafety, GridBoundaryChecksBothAdjacentRows) {
  auto free = [](int x, int y) { return x != 2 || y != 1; };
  EXPECT_FALSE(map_planner_safety::segmentFree(1.5, 2.0, 3.5, 2.0, free));
}

TEST(PathSafety, RejectsNonFiniteAndExcessiveCoordinates) {
  auto free = [](int, int) { return true; };
  EXPECT_FALSE(map_planner_safety::segmentFree(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, 1.0, free));
  EXPECT_FALSE(map_planner_safety::segmentFree(0.0, 0.0, 1e30, 1.0, free));
}

TEST(PathSafety, CoversShortSegmentsAndNegativeCoordinates) {
  std::set<std::pair<int, int>> cells;
  auto free = [&](int x, int y) { cells.emplace(x, y); return true; };
  EXPECT_TRUE(map_planner_safety::segmentFree(-1.1, -2.1, -0.9, -1.9, free));
  EXPECT_EQ(cells.count({-2, -3}), 1u);
  EXPECT_EQ(cells.count({-1, -2}), 1u);
  EXPECT_EQ(cells.count({-2, -2}), 1u);
  EXPECT_EQ(cells.count({-1, -3}), 1u);
}

TEST(PathSafety, BlockedLongRayExitsWithoutAllocatingTheWholeRay) {
  allocation_count = 0;
  track_allocations = true;
  const bool free = map_planner_safety::segmentFree(
      0.5, 0.5, 999999.5, 0.5, [](int, int) { return false; });
  track_allocations = false;
  EXPECT_FALSE(free);
  EXPECT_EQ(allocation_count, 0u);
}

TEST(PathSafety, CellCoverageMatchesPreviousConservativeOracle) {
  unsigned seed = 142546;
  auto coordinate = [&]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<double>(seed % 321) / 20.0 - 8.0;
  };
  for (int i = 0; i < 1000; ++i) {
    double x0 = coordinate(), y0 = coordinate(), x1 = coordinate(), y1 = coordinate();
    if (i % 3 == 0) x0 = std::round(x0);
    if (i % 5 == 0) y1 = std::round(y1);
    if (i % 7 == 0) y1 = y0;
    if (i % 11 == 0) x1 = x0;
    std::set<std::pair<int, int>> current, previous;
    EXPECT_TRUE(map_planner_safety::segmentFree(x0, y0, x1, y1,
        [&](int x, int y) { current.emplace(x, y); return true; }));
    EXPECT_TRUE(previousConservativeTraversal(x0, y0, x1, y1,
        [&](int x, int y) { previous.emplace(x, y); return true; }));
    EXPECT_EQ(current, previous) << "ray=" << x0 << "," << y0 << " -> " << x1 << "," << y1;
  }
}
