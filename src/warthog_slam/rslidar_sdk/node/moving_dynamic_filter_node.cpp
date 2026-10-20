#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct Voxel
{
  int x;
  int y;
  int z;

  bool operator==(const Voxel& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash
{
  std::size_t operator()(const Voxel& voxel) const
  {
    const std::size_t x = std::hash<int>{}(voxel.x);
    const std::size_t y = std::hash<int>{}(voxel.y);
    const std::size_t z = std::hash<int>{}(voxel.z);
    return x ^ (y + 0x9e3779b9U + (x << 6U) + (x >> 2U)) ^
           (z + 0x9e3779b9U + (y << 6U) + (y >> 2U));
  }
};

struct Evidence
{
  ros::Time first_seen;
  ros::Time last_seen;
  int scans = 0;
  bool confirmed = false;
};

struct PendingCloud
{
  sensor_msgs::PointCloud2ConstPtr cloud;
  ros::Time end_stamp;
  bool use_point_time = false;
};

const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                         const std::string& name)
{
  for (const auto& field : cloud.fields)
  {
    if (field.name == name)
    {
      return &field;
    }
  }
  return nullptr;
}

bool validXYZField(const sensor_msgs::PointCloud2& cloud,
                   const sensor_msgs::PointField* field)
{
  return field != nullptr && field->count >= 1U &&
         field->datatype == sensor_msgs::PointField::FLOAT32 &&
         field->offset + sizeof(float) <= cloud.point_step;
}

float readFloat(const std::uint8_t* data)
{
  float value;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

double readDouble(const std::uint8_t* data)
{
  double value;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

tf2::Transform poseTransform(const geometry_msgs::Pose& pose)
{
  const auto& q = pose.orientation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w),
                        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

tf2::Transform stampedTransform(const geometry_msgs::TransformStamped& stamped)
{
  const auto& q = stamped.transform.rotation;
  const auto& t = stamped.transform.translation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w),
                        tf2::Vector3(t.x, t.y, t.z));
}

sensor_msgs::PointCloud2 selectPoints(const sensor_msgs::PointCloud2& input,
                                      const std::vector<std::size_t>& offsets)
{
  sensor_msgs::PointCloud2 output;
  output.header = input.header;
  output.height = 1U;
  output.width = static_cast<std::uint32_t>(offsets.size());
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.point_step * output.width;
  output.is_dense = true;
  output.data.resize(output.row_step);
  std::size_t destination = 0U;
  for (const std::size_t source : offsets)
  {
    std::memcpy(output.data.data() + destination, input.data.data() + source,
                input.point_step);
    destination += input.point_step;
  }
  return output;
}

}  // namespace

class MovingDynamicFilter
{
public:
  MovingDynamicFilter()
    : private_nh_("~"), tf_listener_(tf_buffer_)
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/rslidar_points");
    private_nh_.param<std::string>("odom_topic", odom_topic_,
                                   "/outdoor_waypoint_nav/odometry/filtered");
    private_nh_.param<std::string>("static_topic", static_topic_,
                                   "/rslidar_points_static_moving");
    private_nh_.param<std::string>("dynamic_topic", dynamic_topic_,
                                   "/rslidar_points_dynamic_moving");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param("voxel_size_m", voxel_size_m_, 0.12);
    private_nh_.param("min_static_age_s", min_static_age_s_, 2.0);
    private_nh_.param("min_static_scans", min_static_scans_, 5);
    private_nh_.param("max_observation_gap_s", max_observation_gap_s_, 0.5);
    private_nh_.param("neighbor_support_gap_s", neighbor_support_gap_s_, 1.0);
    private_nh_.param("max_odom_gap_s", max_odom_gap_s_, 0.15);
    private_nh_.param("deskew", deskew_, true);
    private_nh_.param("max_scan_duration_s", max_scan_duration_s_, 0.2);
    private_nh_.param("evidence_timeout_s", evidence_timeout_s_, 5.0);
    private_nh_.param("max_evidence_voxels", max_evidence_voxels_, 500000);

    voxel_size_m_ = std::max(0.02, voxel_size_m_);
    min_static_age_s_ = std::max(0.1, min_static_age_s_);
    min_static_scans_ = std::max(2, min_static_scans_);
    max_observation_gap_s_ = std::max(0.1, max_observation_gap_s_);
    neighbor_support_gap_s_ = std::max(max_observation_gap_s_, neighbor_support_gap_s_);
    max_odom_gap_s_ = std::max(0.01, max_odom_gap_s_);
    max_scan_duration_s_ = std::max(0.01, max_scan_duration_s_);
    evidence_timeout_s_ = std::max(max_observation_gap_s_, evidence_timeout_s_);
    max_evidence_voxels_ = std::max(10000, max_evidence_voxels_);

    static_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_topic_, 2);
    dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_topic_, 2);
    odom_sub_ = nh_.subscribe(odom_topic_, 100, &MovingDynamicFilter::odomCallback, this);
    cloud_sub_ = nh_.subscribe(input_topic_, 3, &MovingDynamicFilter::cloudCallback, this);

    ROS_INFO_STREAM("Moving LiDAR filter: " << input_topic_ << " + " << odom_topic_
                    << " -> " << static_topic_ << " (" << odom_frame_ << " evidence, "
                    << min_static_age_s_ << " s minimum age)");
  }

private:
  bool interpolatePose(const ros::Time& stamp, tf2::Transform& transform) const
  {
    const auto later = std::lower_bound(odometry_.begin(), odometry_.end(), stamp,
      [](const nav_msgs::OdometryConstPtr& sample, const ros::Time& time) {
        return sample->header.stamp < time;
      });
    if (later == odometry_.end())
    {
      return false;
    }
    if ((*later)->header.stamp == stamp)
    {
      transform = poseTransform((*later)->pose.pose);
      return true;
    }
    if (later == odometry_.begin())
    {
      return false;
    }

    const auto earlier = std::prev(later);
    const double gap = ((*later)->header.stamp - (*earlier)->header.stamp).toSec();
    if (gap <= 0.0 || gap > max_odom_gap_s_)
    {
      return false;
    }
    const double fraction = (stamp - (*earlier)->header.stamp).toSec() / gap;
    const tf2::Transform first = poseTransform((*earlier)->pose.pose);
    const tf2::Transform second = poseTransform((*later)->pose.pose);
    const tf2::Vector3 position = first.getOrigin().lerp(second.getOrigin(), fraction);
    tf2::Quaternion orientation = first.getRotation().slerp(second.getRotation(), fraction);
    orientation.normalize();
    transform = tf2::Transform(orientation, position);
    return true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& odom)
  {
    if (odom->header.stamp.isZero() || odom->header.frame_id != odom_frame_ ||
        odom->child_frame_id != base_frame_)
    {
      ROS_WARN_THROTTLE(5.0, "Moving filter needs stamped odometry pose %s -> %s on %s",
                        odom_frame_.c_str(), base_frame_.c_str(), odom_topic_.c_str());
      return;
    }
    const auto& q = odom->pose.pose.orientation;
    const double norm_squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm_squared) || norm_squared < 0.5 || norm_squared > 1.5)
    {
      ROS_WARN_THROTTLE(5.0, "Moving filter received an invalid odometry orientation");
      return;
    }
    if (!odometry_.empty())
    {
      const auto& previous = odometry_.back();
      const double dt = (odom->header.stamp - previous->header.stamp).toSec();
      if (dt <= 0.0 || dt > 2.0)
      {
        ROS_WARN("Moving filter odometry time jumped; clearing background evidence");
        odometry_.clear();
        pending_clouds_.clear();
        evidence_.clear();
        last_cloud_stamp_ = ros::Time(0);
      }
      else
      {
        const tf2::Transform first = poseTransform(previous->pose.pose);
        const tf2::Transform second = poseTransform(odom->pose.pose);
        const double translation = (second.getOrigin() - first.getOrigin()).length();
        const double rotation = first.getRotation().angleShortestPath(second.getRotation());
        if (translation > 1.0 || rotation > 0.7)
        {
          ROS_WARN("Moving filter odometry pose jumped; clearing background evidence");
          evidence_.clear();
        }
      }
    }
    odometry_.push_back(odom);
    while (odometry_.size() > 300U)
    {
      odometry_.pop_front();
    }
    processPending();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if (cloud->header.stamp.isZero())
    {
      ROS_WARN_THROTTLE(5.0, "Moving filter requires a stamped point cloud");
      return;
    }
    PendingCloud pending;
    pending.cloud = cloud;
    pending.end_stamp = cloud->header.stamp;
    const auto* time_field = findField(*cloud, "timestamp");
    if (deskew_ && time_field != nullptr &&
        time_field->datatype == sensor_msgs::PointField::FLOAT64 &&
        time_field->count >= 1U &&
        time_field->offset + sizeof(double) <= cloud->point_step &&
        cloud->row_step >= cloud->width * cloud->point_step &&
        static_cast<std::size_t>(cloud->row_step) * cloud->height <= cloud->data.size())
    {
      pending.use_point_time = true;
      for (std::uint32_t row = 0U; row < cloud->height && pending.use_point_time; ++row)
      {
        for (std::uint32_t col = 0U; col < cloud->width; ++col)
        {
          const std::size_t offset = static_cast<std::size_t>(row) * cloud->row_step +
                                     static_cast<std::size_t>(col) * cloud->point_step;
          const double point_time = readDouble(cloud->data.data() + offset + time_field->offset);
          const double delta = point_time - cloud->header.stamp.toSec();
          if (!std::isfinite(delta) || delta < -0.02 || delta > max_scan_duration_s_)
          {
            pending.use_point_time = false;
            pending.end_stamp = cloud->header.stamp;
            ROS_WARN_THROTTLE(5.0, "Moving filter found invalid point times; using the cloud stamp");
            break;
          }
          if (point_time > pending.end_stamp.toSec())
          {
            pending.end_stamp.fromSec(point_time);
          }
        }
      }
    }
    pending_clouds_.push_back(pending);
    while (pending_clouds_.size() > 4U)
    {
      ROS_WARN_THROTTLE(5.0, "Moving filter dropped a cloud while waiting for odometry");
      pending_clouds_.pop_front();
    }
    processPending();
  }

  void processPending()
  {
    while (!pending_clouds_.empty() && !odometry_.empty())
    {
      const PendingCloud& pending = pending_clouds_.front();
      const auto& cloud = pending.cloud;
      const ros::Time stamp = cloud->header.stamp;
      if (pending.end_stamp > odometry_.back()->header.stamp)
      {
        return;  // Wait for the next odometry pose; never extrapolate motion.
      }
      if (stamp < odometry_.front()->header.stamp)
      {
        ROS_WARN_THROTTLE(5.0, "Moving filter dropped a cloud older than cached odometry");
        pending_clouds_.pop_front();
        continue;
      }
      tf2::Transform odom_to_base;
      if (!interpolatePose(stamp, odom_to_base))
      {
        ROS_WARN_THROTTLE(5.0, "Moving filter dropped a cloud with an odometry timestamp gap");
        pending_clouds_.pop_front();
        continue;
      }
      processCloud(*cloud, odom_to_base, pending.use_point_time);
      pending_clouds_.pop_front();
    }
  }

  bool makeVoxel(const tf2::Vector3& point, Voxel& voxel) const
  {
    const double x = std::floor(point.x() / voxel_size_m_);
    const double y = std::floor(point.y() / voxel_size_m_);
    const double z = std::floor(point.z() / voxel_size_m_);
    if (x < std::numeric_limits<int>::min() || x > std::numeric_limits<int>::max() ||
        y < std::numeric_limits<int>::min() || y > std::numeric_limits<int>::max() ||
        z < std::numeric_limits<int>::min() || z > std::numeric_limits<int>::max())
    {
      return false;
    }
    voxel = Voxel{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
    return true;
  }

  bool hasConfirmedEvidence(const Voxel& voxel, const ros::Time& stamp,
                            double max_gap_s) const
  {
    const auto found = evidence_.find(voxel);
    return found != evidence_.end() && found->second.confirmed &&
           (stamp - found->second.last_seen).toSec() <= max_gap_s;
  }

  bool isSupportedStatic(const Voxel& voxel, const ros::Time& stamp) const
  {
    if (hasConfirmedEvidence(voxel, stamp, evidence_timeout_s_))
    {
      return true;
    }

    // A wall may hit the adjacent voxel as the viewing angle changes. Use
    // only face-adjacent, recently confirmed cells; do not spread evidence
    // diagonally or promote new foreground points from a single scan.
    static constexpr int offsets[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    for (const auto& offset : offsets)
    {
      const Voxel neighbor{voxel.x + offset[0], voxel.y + offset[1],
                           voxel.z + offset[2]};
      if (hasConfirmedEvidence(neighbor, stamp, neighbor_support_gap_s_))
      {
        return true;
      }
    }
    return false;
  }

  void processCloud(const sensor_msgs::PointCloud2& cloud,
                    const tf2::Transform& odom_to_base, bool use_point_time)
  {
    if (!last_cloud_stamp_.isZero() && cloud.header.stamp <= last_cloud_stamp_)
    {
      ROS_WARN_THROTTLE(5.0, "Moving filter dropped an out-of-order cloud");
      return;
    }
    if (!last_cloud_stamp_.isZero() &&
        (cloud.header.stamp - last_cloud_stamp_).toSec() > evidence_timeout_s_)
    {
      evidence_.clear();
    }
    last_cloud_stamp_ = cloud.header.stamp;

    const auto* x_field = findField(cloud, "x");
    const auto* y_field = findField(cloud, "y");
    const auto* z_field = findField(cloud, "z");
    if (cloud.is_bigendian || !validXYZField(cloud, x_field) ||
        !validXYZField(cloud, y_field) || !validXYZField(cloud, z_field) ||
        cloud.point_step == 0U || cloud.row_step < cloud.width * cloud.point_step ||
        static_cast<std::size_t>(cloud.row_step) * cloud.height > cloud.data.size())
    {
      ROS_ERROR_THROTTLE(5.0, "Moving filter needs a little-endian XYZ FLOAT32 PointCloud2");
      return;
    }

    tf2::Transform base_to_lidar;
    base_to_lidar.setIdentity();
    if (cloud.header.frame_id != base_frame_)
    {
      try
      {
        base_to_lidar = stampedTransform(tf_buffer_.lookupTransform(
          base_frame_, cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.05)));
      }
      catch (const tf2::TransformException& error)
      {
        ROS_WARN_THROTTLE(5.0, "Moving filter needs TF %s -> %s: %s",
                          base_frame_.c_str(), cloud.header.frame_id.c_str(), error.what());
        return;
      }
    }
    const tf2::Transform odom_to_lidar = odom_to_base * base_to_lidar;
    const auto* time_field = use_point_time ? findField(cloud, "timestamp") : nullptr;
    std::unordered_map<int, tf2::Transform> scan_transforms;

    // Classify once per voxel, then reuse the answer for points in that cell.
    std::unordered_map<Voxel, bool, VoxelHash> observed_voxels;
    std::vector<std::size_t> static_offsets;
    std::vector<std::size_t> dynamic_offsets;
    static_offsets.reserve(static_cast<std::size_t>(cloud.width) * cloud.height / 2U);
    dynamic_offsets.reserve(static_offsets.capacity());

    for (std::uint32_t row = 0U; row < cloud.height; ++row)
    {
      for (std::uint32_t col = 0U; col < cloud.width; ++col)
      {
        const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
                                   static_cast<std::size_t>(col) * cloud.point_step;
        const std::uint8_t* data = cloud.data.data() + offset;
        const float x = readFloat(data + x_field->offset);
        const float y = readFloat(data + y_field->offset);
        const float z = readFloat(data + z_field->offset);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
          continue;
        }

        const tf2::Transform* point_transform = &odom_to_lidar;
        if (time_field != nullptr)
        {
          const double point_time = readDouble(data + time_field->offset);
          const int millisecond = static_cast<int>(std::floor(
            (point_time - cloud.header.stamp.toSec()) * 1000.0));
          auto cached = scan_transforms.find(millisecond);
          if (cached == scan_transforms.end())
          {
            const ros::Time bin_stamp = cloud.header.stamp + ros::Duration(millisecond * 0.001);
            tf2::Transform bin_pose;
            if (!interpolatePose(bin_stamp, bin_pose))
            {
              dynamic_offsets.push_back(offset);
              continue;
            }
            cached = scan_transforms.emplace(millisecond, bin_pose * base_to_lidar).first;
          }
          point_transform = &cached->second;
        }
        const tf2::Vector3 fixed_point = *point_transform * tf2::Vector3(x, y, z);
        Voxel voxel;
        if (!makeVoxel(fixed_point, voxel))
        {
          continue;
        }
        auto current = observed_voxels.find(voxel);
        if (current == observed_voxels.end())
        {
          current = observed_voxels.emplace(
            voxel, isSupportedStatic(voxel, cloud.header.stamp)).first;
        }
        (current->second ? static_offsets : dynamic_offsets).push_back(offset);
      }
    }

    for (const auto& observation : observed_voxels)
    {
      auto inserted = evidence_.emplace(
        observation.first, Evidence{cloud.header.stamp, cloud.header.stamp, 1, false});
      if (!inserted.second)
      {
        Evidence& state = inserted.first->second;
        const double gap = (cloud.header.stamp - state.last_seen).toSec();
        if (gap > max_observation_gap_s_ &&
            (!state.confirmed || gap > evidence_timeout_s_))
        {
          state = Evidence{cloud.header.stamp, cloud.header.stamp, 1, false};
        }
        else
        {
          state.last_seen = cloud.header.stamp;
          state.scans = std::min(min_static_scans_, state.scans + 1);
          if (state.scans >= min_static_scans_ &&
              (cloud.header.stamp - state.first_seen).toSec() >= min_static_age_s_)
          {
            state.confirmed = true;
          }
        }
      }
    }

    static_pub_.publish(selectPoints(cloud, static_offsets));
    dynamic_pub_.publish(selectPoints(cloud, dynamic_offsets));
    ++processed_clouds_;
    if (processed_clouds_ % 10U == 0U)
    {
      for (auto it = evidence_.begin(); it != evidence_.end();)
      {
        if ((cloud.header.stamp - it->second.last_seen).toSec() > evidence_timeout_s_)
        {
          it = evidence_.erase(it);
        }
        else
        {
          ++it;
        }
      }
      if (evidence_.size() > static_cast<std::size_t>(max_evidence_voxels_))
      {
        // Preserve recent static evidence. Clearing the whole model caused
        // large black regions whenever the robot crossed a new area.
        const std::size_t keep = static_cast<std::size_t>(max_evidence_voxels_) * 4U / 5U;
        std::vector<ros::Time> last_seen;
        last_seen.reserve(evidence_.size());
        for (const auto& state : evidence_)
        {
          last_seen.push_back(state.second.last_seen);
        }
        const std::size_t remove = last_seen.size() - keep;
        std::nth_element(last_seen.begin(), last_seen.begin() + remove, last_seen.end());
        const ros::Time cutoff = last_seen[remove];
        for (auto it = evidence_.begin(); it != evidence_.end() && evidence_.size() > keep;)
        {
          if (it->second.last_seen <= cutoff)
          {
            it = evidence_.erase(it);
          }
          else
          {
            ++it;
          }
        }
        ROS_WARN("Moving filter reached its evidence limit; evicted oldest cells");
      }
    }
    ROS_INFO_THROTTLE(2.0, "Moving filter: input=%zu static=%zu candidate=%zu voxels=%zu",
                      static_offsets.size() + dynamic_offsets.size(), static_offsets.size(),
                      dynamic_offsets.size(), evidence_.size());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher static_pub_;
  ros::Publisher dynamic_pub_;
  std::string input_topic_;
  std::string odom_topic_;
  std::string static_topic_;
  std::string dynamic_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  double voxel_size_m_ = 0.12;
  double min_static_age_s_ = 2.0;
  double max_observation_gap_s_ = 0.5;
  double neighbor_support_gap_s_ = 1.0;
  double max_odom_gap_s_ = 0.15;
  bool deskew_ = true;
  double max_scan_duration_s_ = 0.2;
  double evidence_timeout_s_ = 5.0;
  int min_static_scans_ = 5;
  int max_evidence_voxels_ = 500000;
  std::uint64_t processed_clouds_ = 0U;
  ros::Time last_cloud_stamp_;
  std::deque<nav_msgs::OdometryConstPtr> odometry_;
  std::deque<PendingCloud> pending_clouds_;
  std::unordered_map<Voxel, Evidence, VoxelHash> evidence_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moving_dynamic_filter");
  MovingDynamicFilter filter;
  ros::spin();
  return 0;
}
