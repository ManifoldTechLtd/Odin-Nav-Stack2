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

#include "path_quality.h"

using map_planner_quality::Point2d;
using map_planner_quality::computeGeometryMetrics;
using map_planner_quality::headingErrorDeg;
using map_planner_quality::turnPenaltyCost;

TEST(PathQuality, NegativeYawIsValid) {
  EXPECT_NEAR(headingErrorDeg(-M_PI_2, 0.0, -1.0), 0.0, 1e-9);
  EXPECT_NEAR(headingErrorDeg(-M_PI_2, 1.0, 0.0), 90.0, 1e-9);
}

TEST(PathQuality, StraightPathHasNoSyntheticEndKink) {
  const double angle = -111.0 * M_PI / 180.0;
  std::vector<Point2d> points;
  for (int i = 0; i <= 20; ++i) {
    points.push_back({0.1 * i * std::cos(angle), 0.1 * i * std::sin(angle)});
  }
  const auto metrics = computeGeometryMetrics(points, angle);
  EXPECT_NEAR(metrics.heading_error_deg, 0.0, 1e-6);
  EXPECT_NEAR(metrics.max_kink_deg, 0.0, 1e-6);
}

TEST(PathQuality, RightAngleIsMeasuredFromArcSamples) {
  std::vector<Point2d> points;
  for (int i = 0; i <= 10; ++i) points.push_back({0.1 * i, 0.0});
  for (int i = 1; i <= 10; ++i) points.push_back({1.0, 0.1 * i});
  const auto metrics = computeGeometryMetrics(points, 0.0);
  EXPECT_NEAR(metrics.heading_error_deg, 0.0, 1e-6);
  EXPECT_GT(metrics.max_kink_deg, 80.0);
  EXPECT_LE(metrics.max_kink_deg, 90.0);
}

TEST(PathQuality, DuplicatePointsDoNotCreateKinks) {
  const std::vector<Point2d> points{{0.0, 0.0}, {0.3, 0.0}, {0.3, 0.0},
                                    {0.6, 0.0}, {1.0, 0.0}, {1.0, 0.0}};
  const auto metrics = computeGeometryMetrics(points, 0.0);
  EXPECT_NEAR(metrics.heading_error_deg, 0.0, 1e-6);
  EXPECT_NEAR(metrics.max_kink_deg, 0.0, 1e-6);
}

TEST(PathQuality, TurnPenaltyPreservesStraightMotion) {
  EXPECT_NEAR(turnPenaltyCost(1.0, 0.0, 1.0, 0.0, 0.5), 0.0, 1e-9);
}

TEST(PathQuality, TurnPenaltyIncreasesWithDirectionChange) {
  const double turn_45 = turnPenaltyCost(1.0, 0.0, 1.0, 1.0, 0.5);
  const double turn_90 = turnPenaltyCost(1.0, 0.0, 0.0, 1.0, 0.5);
  const double turn_180 = turnPenaltyCost(1.0, 0.0, -1.0, 0.0, 0.5);
  EXPECT_GT(turn_45, 0.0);
  EXPECT_GT(turn_90, turn_45);
  EXPECT_GT(turn_180, turn_90);
}

TEST(PathQuality, DisabledTurnPenaltyMatchesLegacyCost) {
  EXPECT_NEAR(turnPenaltyCost(1.0, 0.0, 0.0, 1.0, 0.0), 0.0, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
