// Copyright (c) 2025 Kostubh Khandelwal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "smooth_path_tracker/path_processor.hpp"
#include <vector>
#include <cmath>

TEST(PathProcessorTest, Distance) {
    PathProcessor processor;
    Point p1 = {0.0, 0.0};
    Point p2 = {3.0, 4.0};
    EXPECT_DOUBLE_EQ(processor.distance(p1, p2), 5.0);

    // Test with same point
    EXPECT_DOUBLE_EQ(processor.distance(p1, p1), 0.0);

    // Test with negative coordinates
    Point p3 = {-3.0, -4.0};
    EXPECT_DOUBLE_EQ(processor.distance(p1, p3), 5.0);
}

TEST(PathProcessorTest, CubicBezier) {
    PathProcessor processor;
    Point p0 = {0, 0}, p1 = {1, 1}, p2 = {2, -1}, p3 = {3, 0};

    // Test t=0 (start point)
    Point res0 = processor.cubic_bezier(p0, p1, p2, p3, 0.0);
    EXPECT_DOUBLE_EQ(res0.x, p0.x);
    EXPECT_DOUBLE_EQ(res0.y, p0.y);

    // Test t=1 (end point)
    Point res1 = processor.cubic_bezier(p0, p1, p2, p3, 1.0);
    EXPECT_DOUBLE_EQ(res1.x, p3.x);
    EXPECT_DOUBLE_EQ(res1.y, p3.y);

    // Test t=0.5 (midpoint)
    Point res05 = processor.cubic_bezier(p0, p1, p2, p3, 0.5);
    EXPECT_DOUBLE_EQ(res05.x, 1.5);
    EXPECT_DOUBLE_EQ(res05.y, 0.0); // Corrected from 0.125
}

TEST(PathProcessorTest, CalculateControlPoints) {
    PathProcessor processor;
    std::vector<Point> waypoints = {{0, 0}, {1, 1}, {2, 0}};
    std::vector<Point> p1, p2;

    processor.calculate_control_points(waypoints, p1, p2);

    ASSERT_EQ(p1.size(), 2);
    ASSERT_EQ(p2.size(), 2);

    // Corrected expected values
    EXPECT_NEAR(p1[0].x, 0.333333, 1e-5);
    EXPECT_NEAR(p1[0].y, 0.5, 1e-5);
    EXPECT_NEAR(p2[0].x, 0.666667, 1e-5);
    EXPECT_NEAR(p2[0].y, 1.0, 1e-5);
    EXPECT_NEAR(p1[1].x, 1.333333, 1e-5);
    EXPECT_NEAR(p1[1].y, 1.0, 1e-5);
    EXPECT_NEAR(p2[1].x, 1.666667, 1e-5);
    EXPECT_NEAR(p2[1].y, 0.5, 1e-5);

    // Edge case: 2 waypoints
    waypoints = {{0, 0}, {1, 1}};
    // Clear vectors to avoid carrying over data from previous test
    p1.clear();
    p2.clear();
    processor.calculate_control_points(waypoints, p1, p2);
    ASSERT_EQ(p1.size(), 1);
    ASSERT_EQ(p2.size(), 1);
    EXPECT_NEAR(p1[0].x, 1.0/3.0, 1e-6);
    EXPECT_NEAR(p1[0].y, 1.0/3.0, 1e-6);
    EXPECT_NEAR(p2[0].x, 2.0/3.0, 1e-6);
    EXPECT_NEAR(p2[0].y, 2.0/3.0, 1e-6);

    // Edge case: empty waypoints
    waypoints.clear();
    p1.clear();
    p2.clear();
    processor.calculate_control_points(waypoints, p1, p2);
    EXPECT_TRUE(p1.empty());
    EXPECT_TRUE(p2.empty());

    // Edge case: one waypoint
    waypoints = {{0,0}};
    p1.clear();
    p2.clear();
    processor.calculate_control_points(waypoints, p1, p2);
    EXPECT_TRUE(p1.empty());
    EXPECT_TRUE(p2.empty());
}

TEST(PathProcessorTest, GenerateSmoothPath) {
    PathProcessor processor;
    std::vector<Point> waypoints = {{0, 0}, {10, 0}};
    double points_per_meter = 10.0;

    std::vector<Point> smooth_path = processor.generate_smooth_path(waypoints, points_per_meter);

    // For a straight line, the number of points should be close to length * points_per_meter
    EXPECT_NEAR(smooth_path.size(), 10 * 10, 1);
    // First and last points should be the same
    EXPECT_DOUBLE_EQ(smooth_path.front().x, waypoints.front().x);
    EXPECT_DOUBLE_EQ(smooth_path.front().y, waypoints.front().y);
    EXPECT_DOUBLE_EQ(smooth_path.back().x, waypoints.back().x);
    EXPECT_DOUBLE_EQ(smooth_path.back().y, waypoints.back().y);

    // Edge case: empty waypoints
    waypoints.clear();
    smooth_path = processor.generate_smooth_path(waypoints, points_per_meter);
    EXPECT_TRUE(smooth_path.empty());

    // Edge case: one waypoint
    waypoints = {{0,0}};
    smooth_path = processor.generate_smooth_path(waypoints, points_per_meter);
    EXPECT_TRUE(smooth_path.empty());
}

TEST(PathProcessorTest, GenerateTimedPath) {
    PathProcessor processor;
    std::vector<Point> smooth_points = {{0, 0}, {1, 0}, {2, 0}};
    double desired_velocity = 0.5;

    std::vector<TimedPose> timed_path = processor.generate_timed_path(smooth_points, desired_velocity);

    ASSERT_EQ(timed_path.size(), 3);

    // Check time offsets
    EXPECT_DOUBLE_EQ(timed_path[0].time_offset_secs, 0.0);
    EXPECT_DOUBLE_EQ(timed_path[1].time_offset_secs, 2.0); // 1m / 0.5m/s
    EXPECT_DOUBLE_EQ(timed_path[2].time_offset_secs, 4.0); // 1m / 0.5m/s + 2.0s

    // Check yaws
    EXPECT_DOUBLE_EQ(timed_path[0].yaw, 0.0); // atan2(0, 1)
    EXPECT_DOUBLE_EQ(timed_path[1].yaw, 0.0); // atan2(0, 1)
    EXPECT_DOUBLE_EQ(timed_path[2].yaw, 0.0); // same as last

    // Check computed velocity
    for (size_t i = 1; i < timed_path.size(); ++i) {
        Point p_prev = {timed_path[i-1].x, timed_path[i-1].y};
        Point p_curr = {timed_path[i].x, timed_path[i].y};
        double dist = processor.distance(p_prev, p_curr);
        double time_delta = timed_path[i].time_offset_secs - timed_path[i-1].time_offset_secs;
        // Avoid division by zero
        if (time_delta > 1e-9) {
            double computed_velocity = dist / time_delta;
            EXPECT_NEAR(computed_velocity, desired_velocity, 1e-6);
        } else {
            // If time delta is zero, distance should also be zero
            EXPECT_NEAR(dist, 0.0, 1e-6);
        }
    }

    // Edge case: empty path
    smooth_points.clear();
    timed_path = processor.generate_timed_path(smooth_points, desired_velocity);
    EXPECT_TRUE(timed_path.empty());

    // Edge case: one point
    smooth_points = {{1,1}};
    timed_path = processor.generate_timed_path(smooth_points, desired_velocity);
    ASSERT_EQ(timed_path.size(), 1);
    EXPECT_DOUBLE_EQ(timed_path[0].x, 1.0);
    EXPECT_DOUBLE_EQ(timed_path[0].y, 1.0);
    EXPECT_DOUBLE_EQ(timed_path[0].time_offset_secs, 0.0);
    EXPECT_DOUBLE_EQ(timed_path[0].yaw, 0.0);
}
