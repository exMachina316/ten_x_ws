#ifndef PATH_PROCESSOR_HPP_
#define PATH_PROCESSOR_HPP_

#include <cmath>
#include <vector>

// A simple 2D point structure
struct Point {
  double x, y;
};

// A structure to hold timed pose information
struct TimedPose {
  double x;
  double y;
  double yaw;
  double time_offset_secs;
};

/**
 * @brief A class to process a path, including smoothing, sampling, and time
 * parameterization.
 *
 * This class encapsulates the logic for taking a set of coarse waypoints and
 * generating a smooth, timed path. It is independent of ROS.
 */
class PathProcessor {
public:
  /**
   * @brief Generates a timed, smooth path from a given set of waypoints.
   *
   * @param waypoints A vector of Point structures representing the coarse
   * waypoints.
   * @param points_per_meter The desired number of points per meter of path
   * length.
   * @param desired_velocity The desired velocity for the path.
   * @return std::vector<TimedPose> A vector of TimedPose structures
   * representing the timed path.
   */
  std::vector<TimedPose>
  generate_timed_path(const std::vector<Point> &waypoints,
                      double points_per_meter, double desired_velocity);

private:
  /**
   * @brief Generates a smooth path from a given set of waypoints.
   *
   * @param waypoints A vector of Point structures representing the coarse
   * waypoints.
   * @param points_per_meter The desired number of points per meter of path
   * length.
   * @return std::vector<Point> A vector of Point structures representing the
   * smooth path.
   */
  std::vector<Point> generate_smooth_path(const std::vector<Point> &waypoints,
                                          double points_per_meter);

  /**
   * @brief Calculates a point on a cubic Bezier curve.
   *
   * @param p0 Start point of the segment.
   * @param p1 First control point.
   * @param p2 Second control point.
   * @param p3 End point of the segment.
   * @param t Parameter value between 0 and 1.
   * @return Point The calculated point on the curve.
   */
  Point cubic_bezier(Point p0, Point p1, Point p2, Point p3, double t);

  /**
   * @brief Calculates the control points for C1 continuous cubic Bezier curves.
   *
   * @param waypoints The input waypoints.
   * @param p1_points The first control points (output).
   * @param p2_points The second control points (output).
   */
  void calculate_control_points(const std::vector<Point> &waypoints,
                                std::vector<Point> &p1_points,
                                std::vector<Point> &p2_points);

  /**
   * @brief Calculates the distance between two points.
   *
   * @param p1 The first point.
   * @param p2 The second point.
   * @return double The distance between the two points.
   */
  double distance(const Point &p1, const Point &p2);
};

#endif // PATH_PROCESSOR_HPP_
