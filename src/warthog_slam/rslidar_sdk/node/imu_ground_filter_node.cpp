#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Point
{
  float x;
  float y;
  float z;
  double level_x;
  double level_y;
  double level_z;
  double radius;
  int segment;
  int bin;
};

double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

tf2::Quaternion normalizedQuaternion(double x, double y, double z, double w)
{
  tf2::Quaternion quaternion(x, y, z, w);
  const double length_squared = quaternion.length2();
  if (!std::isfinite(length_squared) || length_squared < 1.0e-12)
  {
    throw std::runtime_error("invalid zero-length quaternion");
  }
  quaternion.normalize();
  return quaternion;
}

tf2::Quaternion rotationAligningVectorToZ(tf2::Vector3 source)
{
  if (!std::isfinite(source.length2()) || source.length2() < 1.0e-12)
  {
    throw std::runtime_error("invalid gravity vector");
  }
  source.normalize();
  const tf2::Vector3 target(0.0, 0.0, 1.0);
  const double cosine = clampUnit(source.dot(target));
  if (cosine > 1.0 - 1.0e-10)
  {
    return tf2::Quaternion::getIdentity();
  }
  if (cosine < -1.0 + 1.0e-10)
  {
    return tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), kPi);
  }
  tf2::Vector3 axis = source.cross(target);
  axis.normalize();
  return tf2::Quaternion(axis, std::acos(cosine));
}
}  // namespace

class ImuGroundFilter
{
public:
  ImuGroundFilter()
    : nh_()
    , private_nh_("~")
    , tf_buffer_(ros::Duration(5.0))
    , tf_listener_(tf_buffer_)
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/rslidar_points");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/imu/data_new");
    private_nh_.param<std::string>("obstacle_topic", obstacle_topic_,
                                   "/rslidar_points_obstacles");
    private_nh_.param<std::string>("ground_topic", ground_topic_,
                                   "/rslidar_points_ground");

    private_nh_.param("lidar_height", lidar_height_, 1.23);
    private_nh_.param("min_range", min_range_, 0.50);
    private_nh_.param("max_range", max_range_, 20.0);
    private_nh_.param("min_obstacle_height", min_obstacle_height_, 0.30);
    private_nh_.param("max_obstacle_height", max_obstacle_height_, 1.60);

    private_nh_.param("n_segments", n_segments_, 360);
    private_nh_.param("n_bins", n_bins_, 120);
    private_nh_.param("max_slope", max_slope_, 0.35);
    private_nh_.param("max_dist_to_line", max_dist_to_line_, 0.08);
    private_nh_.param("max_start_height", max_start_height_, 0.20);
    private_nh_.param("ground_seed_max_range", ground_seed_max_range_, 2.0);
    private_nh_.param("max_long_height", max_long_height_, 0.15);
    private_nh_.param("long_threshold", long_threshold_, 1.0);
    private_nh_.param("max_fit_error", max_fit_error_, 0.05);
    private_nh_.param("fit_window", fit_window_, 6);

    private_nh_.param("require_imu", require_imu_, true);
    private_nh_.param("imu_timeout", imu_timeout_, 0.50);
    private_nh_.param("tf_timeout", tf_timeout_, 0.03);
    private_nh_.param("assume_aligned_imu_lidar", assume_aligned_imu_lidar_, true);

    private_nh_.param("self_filter_enabled", self_filter_enabled_, true);
    private_nh_.param("self_x_min", self_x_min_, -0.84);
    private_nh_.param("self_x_max", self_x_max_, 0.84);
    private_nh_.param("self_y_min", self_y_min_, -0.77);
    private_nh_.param("self_y_max", self_y_max_, 0.77);
    private_nh_.param("self_z_min", self_z_min_, -lidar_height_ - 0.10);
    private_nh_.param("self_z_max", self_z_max_, 0.10);

    validateParameters();

    obstacle_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_topic_, 1);
    ground_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_, 1);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 20, &ImuGroundFilter::imuCallback, this);
    cloud_subscriber_ = nh_.subscribe(input_topic_, 1, &ImuGroundFilter::cloudCallback, this,
                                      ros::TransportHints().tcpNoDelay(true));

    ROS_INFO_STREAM("[imu_ground_filter] cloud=" << input_topic_ << " imu=" << imu_topic_
                                                   << " -> obstacles=" << obstacle_topic_
                                                   << " ground=" << ground_topic_);
    ROS_INFO_STREAM("[imu_ground_filter] lidar_height=" << lidar_height_
                                                         << "m slope<=" << max_slope_ << " ("
                                                         << std::atan(max_slope_) * 180.0 / kPi
                                                         << "deg) obstacle_height="
                                                         << min_obstacle_height_ << ".."
                                                         << max_obstacle_height_ << "m sectors="
                                                         << n_segments_ << " bins=" << n_bins_);
  }

private:
  void validateParameters() const
  {
    if (lidar_height_ <= 0.0)
      throw std::invalid_argument("lidar_height must be positive");
    if (min_range_ < 0.0 || max_range_ <= min_range_)
      throw std::invalid_argument("max_range must be greater than min_range");
    if (min_obstacle_height_ < 0.0 || max_obstacle_height_ <= min_obstacle_height_)
      throw std::invalid_argument("invalid obstacle height range");
    if (n_segments_ < 4 || n_bins_ < 2)
      throw std::invalid_argument("n_segments and n_bins are too small");
    if (max_slope_ <= 0.0 || max_dist_to_line_ <= 0.0)
      throw std::invalid_argument("ground thresholds must be positive");
    if (ground_seed_max_range_ <= min_range_)
      throw std::invalid_argument("ground_seed_max_range must exceed min_range");
    if (fit_window_ < 2)
      throw std::invalid_argument("fit_window must be at least two");
    if (imu_timeout_ <= 0.0)
      throw std::invalid_argument("imu_timeout must be positive");
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& message)
  {
    if (message->orientation_covariance[0] < 0.0)
    {
      ROS_ERROR_THROTTLE(2.0, "[imu_ground_filter] %s does not provide orientation",
                         imu_topic_.c_str());
      return;
    }
    try
    {
      const tf2::Quaternion quaternion =
          normalizedQuaternion(message->orientation.x, message->orientation.y,
                               message->orientation.z, message->orientation.w);
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_orientation_ = quaternion;
      imu_frame_ = stripLeadingSlash(message->header.frame_id);
      imu_stamp_ = message->header.stamp;
      imu_receive_time_ = ros::Time::now();
      have_imu_ = true;
    }
    catch (const std::exception& error)
    {
      ROS_ERROR_THROTTLE(2.0, "[imu_ground_filter] %s", error.what());
    }
  }

  static std::string stripLeadingSlash(const std::string& frame)
  {
    if (!frame.empty() && frame.front() == '/')
      return frame.substr(1);
    return frame;
  }

  bool imuToCloudRotation(const std::string& cloud_frame, const std::string& imu_frame,
                          tf2::Quaternion& rotation)
  {
    if (cloud_frame.empty() || imu_frame.empty() || cloud_frame == imu_frame)
    {
      rotation = tf2::Quaternion::getIdentity();
      return true;
    }

    const std::string cache_key = cloud_frame + "\n" + imu_frame;
    const auto cached = frame_rotation_cache_.find(cache_key);
    if (cached != frame_rotation_cache_.end())
    {
      rotation = cached->second;
      return true;
    }

    try
    {
      const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
          cloud_frame, imu_frame, ros::Time(0), ros::Duration(tf_timeout_));
      rotation = normalizedQuaternion(transform.transform.rotation.x,
                                      transform.transform.rotation.y,
                                      transform.transform.rotation.z,
                                      transform.transform.rotation.w);
      frame_rotation_cache_[cache_key] = rotation;
      return true;
    }
    catch (const tf2::TransformException& error)
    {
      if (assume_aligned_imu_lidar_)
      {
        ROS_WARN_THROTTLE(5.0,
                          "[imu_ground_filter] no rotation TF %s <- %s (%s); assuming "
                          "parallel IMU/LiDAR axes",
                          cloud_frame.c_str(), imu_frame.c_str(), error.what());
        rotation = tf2::Quaternion::getIdentity();
        return true;
      }
      ROS_WARN_THROTTLE(2.0, "[imu_ground_filter] cannot transform IMU axis %s -> %s: %s",
                        imu_frame.c_str(), cloud_frame.c_str(), error.what());
      return false;
    }
  }

  bool levellingRotation(const std::string& raw_cloud_frame, tf2::Quaternion& rotation,
                         bool& imu_used, double& tilt_degrees)
  {
    tf2::Quaternion imu_orientation;
    std::string imu_frame;
    ros::Time imu_stamp;
    ros::Time receive_time;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (!have_imu_)
      {
        if (require_imu_)
        {
          ROS_WARN_THROTTLE(2.0, "[imu_ground_filter] waiting for IMU orientation; cloud held");
          return false;
        }
        rotation = tf2::Quaternion::getIdentity();
        imu_used = false;
        tilt_degrees = 0.0;
        return true;
      }
      imu_orientation = imu_orientation_;
      imu_frame = imu_frame_;
      imu_stamp = imu_stamp_;
      receive_time = imu_receive_time_;
    }

    const ros::Time now = ros::Time::now();
    const double receive_age = std::max(0.0, (now - receive_time).toSec());
    const double stamp_age = imu_stamp.isZero() ? 0.0 : std::max(0.0, (now - imu_stamp).toSec());
    const double age = std::max(receive_age, stamp_age);
    if (age > imu_timeout_)
    {
      if (require_imu_)
      {
        ROS_WARN_THROTTLE(2.0, "[imu_ground_filter] IMU orientation is stale (%.3fs); cloud held",
                          age);
        return false;
      }
      rotation = tf2::Quaternion::getIdentity();
      imu_used = false;
      tilt_degrees = 0.0;
      return true;
    }

    tf2::Quaternion cloud_from_imu;
    const std::string cloud_frame = stripLeadingSlash(raw_cloud_frame);
    if (!imuToCloudRotation(cloud_frame, imu_frame, cloud_from_imu))
      return false;

    const tf2::Vector3 world_up(0.0, 0.0, 1.0);
    const tf2::Vector3 up_in_imu = tf2::quatRotate(imu_orientation.inverse(), world_up);
    tf2::Vector3 up_in_cloud = tf2::quatRotate(cloud_from_imu, up_in_imu);
    up_in_cloud.normalize();
    try
    {
      rotation = rotationAligningVectorToZ(up_in_cloud);
    }
    catch (const std::exception& error)
    {
      ROS_WARN_THROTTLE(2.0, "[imu_ground_filter] %s; cloud held", error.what());
      return false;
    }
    imu_used = true;
    tilt_degrees = std::acos(clampUnit(up_in_cloud.z())) * 180.0 / kPi;
    return true;
  }

  bool insideRobot(float x, float y, float z) const
  {
    return self_filter_enabled_ && x >= self_x_min_ && x <= self_x_max_ &&
           y >= self_y_min_ && y <= self_y_max_ && z >= self_z_min_ && z <= self_z_max_;
  }

  bool readAndLevelPoints(const sensor_msgs::PointCloud2ConstPtr& message,
                          const tf2::Quaternion& levelling_rotation,
                          std::vector<Point>& points) const
  {
    points.clear();
    points.reserve(static_cast<std::size_t>(message->width) * message->height);
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*message, "z");
      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator, ++z_iterator)
      {
        const float x = *x_iterator;
        const float y = *y_iterator;
        const float z = *z_iterator;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || insideRobot(x, y, z))
          continue;

        const tf2::Vector3 level = tf2::quatRotate(levelling_rotation, tf2::Vector3(x, y, z));
        const double radius = std::hypot(level.x(), level.y());
        if (radius < min_range_ || radius > max_range_)
          continue;

        int segment = static_cast<int>(
            std::floor((std::atan2(level.y(), level.x()) + kPi) * n_segments_ / (2.0 * kPi)));
        segment = std::max(0, std::min(n_segments_ - 1, segment));
        int bin = static_cast<int>(
            std::floor((radius - min_range_) * n_bins_ / (max_range_ - min_range_)));
        bin = std::max(0, std::min(n_bins_ - 1, bin));
        points.push_back({ x, y, z, level.x(), level.y(), level.z(), radius, segment, bin });
      }
      return true;
    }
    catch (const std::runtime_error& error)
    {
      ROS_ERROR_THROTTLE(2.0, "[imu_ground_filter] invalid PointCloud2: %s", error.what());
      return false;
    }
  }

  void buildGroundModel(const std::vector<Point>& points, std::vector<double>& model_slope,
                        std::vector<double>& model_intercept) const
  {
    const std::size_t cell_count = static_cast<std::size_t>(n_segments_) * n_bins_;
    const double infinity = std::numeric_limits<double>::infinity();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> minimum_z(cell_count, infinity);
    std::vector<double> minimum_radius(cell_count, not_a_number);

    for (const Point& point : points)
    {
      const std::size_t cell = static_cast<std::size_t>(point.segment) * n_bins_ + point.bin;
      if (point.level_z < minimum_z[cell])
      {
        minimum_z[cell] = point.level_z;
        minimum_radius[cell] = point.radius;
      }
    }

    model_slope.assign(cell_count, not_a_number);
    model_intercept.assign(cell_count, not_a_number);
    const double bin_size = (max_range_ - min_range_) / n_bins_;

    std::vector<double> anchor_radius;
    std::vector<double> anchor_height;
    anchor_radius.reserve(static_cast<std::size_t>(fit_window_));
    anchor_height.reserve(static_cast<std::size_t>(fit_window_));

    for (int segment = 0; segment < n_segments_; ++segment)
    {
      double last_radius = 0.0;
      double last_height = -lidar_height_;
      double slope = 0.0;
      bool have_ground_anchor = false;
      anchor_radius.clear();
      anchor_height.clear();

      for (int bin = 0; bin < n_bins_; ++bin)
      {
        const std::size_t cell = static_cast<std::size_t>(segment) * n_bins_ + bin;
        if (!std::isfinite(minimum_z[cell]))
          continue;

        const double radius = minimum_radius[cell];
        const double candidate_z = minimum_z[cell];
        const double radial_gap = std::max(radius - last_radius, bin_size);
        const double predicted_z = last_height + slope * radial_gap;
        double candidate_slope = slope;
        bool accepted = false;

        if (!have_ground_anchor)
        {
          const double height_at_start = candidate_z + lidar_height_;
          candidate_slope = height_at_start / std::max(radius, 1.0e-6);
          accepted = std::abs(candidate_slope) <= max_slope_ &&
                     (radius <= ground_seed_max_range_ ||
                      std::abs(height_at_start) <= max_start_height_);
        }
        else
        {
          const double local_slope = (candidate_z - last_height) / radial_gap;
          const double residual_limit = radial_gap > long_threshold_
                                            ? max_long_height_
                                            : max_dist_to_line_ + max_fit_error_ +
                                                  max_slope_ * std::min(radial_gap, 0.25);
          accepted = std::abs(local_slope) <= max_slope_ &&
                     std::abs(candidate_z - predicted_z) <= residual_limit;
          candidate_slope = local_slope;
        }

        if (accepted)
        {
          have_ground_anchor = true;
          anchor_radius.push_back(radius);
          anchor_height.push_back(candidate_z);
          if (static_cast<int>(anchor_radius.size()) > fit_window_)
          {
            anchor_radius.erase(anchor_radius.begin());
            anchor_height.erase(anchor_height.begin());
          }

          if (anchor_radius.size() >= 2)
          {
            const double sample_count = static_cast<double>(anchor_radius.size());
            double mean_radius = 0.0;
            double mean_height = 0.0;
            for (std::size_t index = 0; index < anchor_radius.size(); ++index)
            {
              mean_radius += anchor_radius[index];
              mean_height += anchor_height[index];
            }
            mean_radius /= sample_count;
            mean_height /= sample_count;

            double numerator = 0.0;
            double denominator = 0.0;
            for (std::size_t index = 0; index < anchor_radius.size(); ++index)
            {
              const double radius_error = anchor_radius[index] - mean_radius;
              numerator += radius_error * (anchor_height[index] - mean_height);
              denominator += radius_error * radius_error;
            }
            const double fitted_slope = denominator > 1.0e-12 ? numerator / denominator : 0.0;
            const double fitted_intercept = mean_height - fitted_slope * mean_radius;
            double fit_error = 0.0;
            for (std::size_t index = 0; index < anchor_radius.size(); ++index)
            {
              fit_error = std::max(
                  fit_error,
                  std::abs(fitted_slope * anchor_radius[index] + fitted_intercept -
                           anchor_height[index]));
            }
            slope = std::abs(fitted_slope) <= max_slope_ &&
                            fit_error <= max_fit_error_ + max_dist_to_line_
                        ? fitted_slope
                        : candidate_slope;
          }
          else
          {
            slope = candidate_slope;
          }
          last_radius = radius;
          last_height = candidate_z;
          model_slope[cell] = slope;
          model_intercept[cell] = candidate_z - slope * radius;
        }
        else
        {
          model_slope[cell] = slope;
          model_intercept[cell] = last_height - slope * last_radius;
        }
      }
    }
  }

  static sensor_msgs::PointCloud2 buildCloud(const std_msgs::Header& header,
                                              const std::vector<Point>& source,
                                              const std::vector<std::size_t>& indices)
  {
    sensor_msgs::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(indices.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(output, "z");
    for (const std::size_t index : indices)
    {
      *x_iterator = source[index].x;
      *y_iterator = source[index].y;
      *z_iterator = source[index].z;
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
    }
    output.is_dense = true;
    return output;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
  {
    tf2::Quaternion levelling_rotation;
    bool imu_used = false;
    double tilt_degrees = 0.0;
    if (!levellingRotation(message->header.frame_id, levelling_rotation, imu_used, tilt_degrees))
      return;

    std::vector<Point> points;
    if (!readAndLevelPoints(message, levelling_rotation, points))
      return;

    std::vector<double> model_slope;
    std::vector<double> model_intercept;
    buildGroundModel(points, model_slope, model_intercept);

    std::vector<std::size_t> ground_indices;
    std::vector<std::size_t> obstacle_indices;
    ground_indices.reserve(points.size() / 2);
    obstacle_indices.reserve(points.size() / 8);
    for (std::size_t index = 0; index < points.size(); ++index)
    {
      const Point& point = points[index];
      const std::size_t cell = static_cast<std::size_t>(point.segment) * n_bins_ + point.bin;
      double ground_z = -lidar_height_;
      if (std::isfinite(model_slope[cell]) && std::isfinite(model_intercept[cell]))
        ground_z = model_slope[cell] * point.radius + model_intercept[cell];
      const double height = point.level_z - ground_z;
      if (std::abs(height) <= max_dist_to_line_)
        ground_indices.push_back(index);
      if (height >= min_obstacle_height_ && height <= max_obstacle_height_)
        obstacle_indices.push_back(index);
    }

    ground_publisher_.publish(buildCloud(message->header, points, ground_indices));
    obstacle_publisher_.publish(buildCloud(message->header, points, obstacle_indices));
    ROS_INFO_THROTTLE(2.0,
                      "[imu_ground_filter] input=%u cropped=%zu ground=%zu obstacles=%zu "
                      "imu=%s tilt=%.1fdeg",
                      message->width * message->height, points.size(), ground_indices.size(),
                      obstacle_indices.size(), imu_used ? "yes" : "fallback", tilt_degrees);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher obstacle_publisher_;
  ros::Publisher ground_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_topic_;
  std::string imu_topic_;
  std::string obstacle_topic_;
  std::string ground_topic_;
  double lidar_height_;
  double min_range_;
  double max_range_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  int n_segments_;
  int n_bins_;
  double max_slope_;
  double max_dist_to_line_;
  double max_start_height_;
  double ground_seed_max_range_;
  double max_long_height_;
  double long_threshold_;
  double max_fit_error_;
  int fit_window_;
  bool require_imu_;
  double imu_timeout_;
  double tf_timeout_;
  bool assume_aligned_imu_lidar_;
  bool self_filter_enabled_;
  double self_x_min_;
  double self_x_max_;
  double self_y_min_;
  double self_y_max_;
  double self_z_min_;
  double self_z_max_;

  std::mutex imu_mutex_;
  bool have_imu_ = false;
  tf2::Quaternion imu_orientation_ = tf2::Quaternion::getIdentity();
  std::string imu_frame_;
  ros::Time imu_stamp_;
  ros::Time imu_receive_time_;
  std::unordered_map<std::string, tf2::Quaternion> frame_rotation_cache_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_ground_filter");
  try
  {
    ImuGroundFilter filter;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("[imu_ground_filter] startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
