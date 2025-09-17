#include "smooth_path_tracker/path_processor.hpp"

std::vector<TimedPose>
PathProcessor::generate_timed_path(const std::vector<Point> &waypoints,
                                   double points_per_meter,
                                   double desired_velocity) {
  std::vector<Point> smooth_points =
      generate_smooth_path(waypoints, points_per_meter);
  std::vector<TimedPose> timed_poses;

  if (smooth_points.empty()) {
    return timed_poses;
  }

  double time_offset_secs = 0.0;

  for (size_t i = 0; i < smooth_points.size(); ++i) {
    if (i > 0) {
      double dist = distance(smooth_points[i - 1], smooth_points[i]);
      double dt = dist / desired_velocity;
      time_offset_secs += dt;
    }

    TimedPose timed_pose;
    timed_pose.x = smooth_points[i].x;
    timed_pose.y = smooth_points[i].y;
    timed_pose.time_offset_secs = time_offset_secs;

    if (i < smooth_points.size() - 1) {
      double dx = smooth_points[i + 1].x - smooth_points[i].x;
      double dy = smooth_points[i + 1].y - smooth_points[i].y;
      timed_pose.yaw = atan2(dy, dx);
    } else {
      if (i > 0) {
        timed_pose.yaw = timed_poses.back().yaw;
      } else {
        timed_pose.yaw = 0.0;
      }
    }
    timed_poses.push_back(timed_pose);
  }
  return timed_poses;
}

std::vector<Point>
PathProcessor::generate_smooth_path(const std::vector<Point> &waypoints,
                                    double points_per_meter) {
  std::vector<Point> smooth_path;
  if (waypoints.size() < 2) {
    return smooth_path;
  }

  std::vector<Point> p1_points, p2_points;
  calculate_control_points(waypoints, p1_points, p2_points);

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    Point p0 = waypoints[i];
    Point p3 = waypoints[i + 1];
    Point p1 = p1_points[i];
    Point p2 = p2_points[i];

    double segment_length = 0;
    Point prev_point = p0;
    for (int j = 1; j <= 10; ++j) {
      double t = static_cast<double>(j) / 10.0;
      Point current_point = cubic_bezier(p0, p1, p2, p3, t);
      segment_length += distance(prev_point, current_point);
      prev_point = current_point;
    }

    int num_points_per_segment =
        std::max(2, static_cast<int>(segment_length * points_per_meter));

    for (int j = 0; j < num_points_per_segment; ++j) {
      double t = static_cast<double>(j) / (num_points_per_segment - 1);
      smooth_path.push_back(cubic_bezier(p0, p1, p2, p3, t));
    }
  }
  return smooth_path;
}

Point PathProcessor::cubic_bezier(Point p0, Point p1, Point p2, Point p3,
                                  double t) {
  Point p;
  double u = 1.0 - t;
  double tt = t * t;
  double uu = u * u;
  double uuu = uu * u;
  double ttt = tt * t;

  p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
  p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;

  return p;
}

void PathProcessor::calculate_control_points(
    const std::vector<Point> &waypoints, std::vector<Point> &p1_points,
    std::vector<Point> &p2_points) {
  int n = waypoints.size() - 1;
  if (n < 1)
    return;

  p1_points.resize(n);
  p2_points.resize(n);

  if (n == 1) {
    p1_points[0] = {(2 * waypoints[0].x + waypoints[1].x) / 3.0,
                    (2 * waypoints[0].y + waypoints[1].y) / 3.0};
    p2_points[0] = {(waypoints[0].x + 2 * waypoints[1].x) / 3.0,
                    (waypoints[0].y + 2 * waypoints[1].y) / 3.0};
    return;
  }

  std::vector<double> a(n), b(n), c(n);
  std::vector<Point> r(n);

  a[0] = 0;
  b[0] = 2;
  c[0] = 1;
  r[0] = {waypoints[0].x + 2 * waypoints[1].x,
          waypoints[0].y + 2 * waypoints[1].y};

  for (int i = 1; i < n - 1; ++i) {
    a[i] = 1;
    b[i] = 4;
    c[i] = 1;
    r[i] = {4 * waypoints[i].x + 2 * waypoints[i + 1].x,
            4 * waypoints[i].y + 2 * waypoints[i + 1].y};
  }

  a[n - 1] = 2;
  b[n - 1] = 7;
  c[n - 1] = 0;
  r[n - 1] = {8 * waypoints[n - 1].x + waypoints[n].x,
              8 * waypoints[n - 1].y + waypoints[n].y};

  for (int i = 1; i < n; ++i) {
    double m = a[i] / b[i - 1];
    b[i] = b[i] - m * c[i - 1];
    r[i] = {r[i].x - m * r[i - 1].x, r[i].y - m * r[i - 1].y};
  }

  p1_points[n - 1] = {r[n - 1].x / b[n - 1], r[n - 1].y / b[n - 1]};
  for (int i = n - 2; i >= 0; --i) {
    p1_points[i] = {(r[i].x - c[i] * p1_points[i + 1].x) / b[i],
                    (r[i].y - c[i] * p1_points[i + 1].y) / b[i]};
  }

  for (int i = 0; i < n - 1; ++i) {
    p2_points[i] = {2 * waypoints[i + 1].x - p1_points[i + 1].x,
                    2 * waypoints[i + 1].y - p1_points[i + 1].y};
  }
  p2_points[n - 1] = {(waypoints[n].x + p1_points[n - 1].x) / 2.0,
                      (waypoints[n].y + p1_points[n - 1].y) / 2.0};
}

double PathProcessor::distance(const Point &p1, const Point &p2) {
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}
