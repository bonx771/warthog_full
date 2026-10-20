#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;

struct BeamKey
{
  int channel;
  int azimuth;

  bool operator==(const BeamKey& other) const
  {
    return channel == other.channel && azimuth == other.azimuth;
  }
};

struct BeamKeyHash
{
  std::size_t operator()(const BeamKey& key) const
  {
    const std::size_t h1 = std::hash<int>{}(key.channel);
    const std::size_t h2 = std::hash<int>{}(key.azimuth);
    return h1 ^ (h2 + 0x9e3779b9U + (h1 << 6U) + (h1 >> 2U));
  }
};

struct CellState
{
  double background_range = 0.0;
  int stable_hits = 0;
  double candidate_range = 0.0;
  int candidate_hits = 0;
  std::uint64_t last_seen_frame = 0;
};

struct Observation
{
  BeamKey key;
  double range;
  std::size_t data_offset;
};

template <typename T>
T readValue(const std::uint8_t* data)
{
  T value;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

std::size_t fieldSize(const sensor_msgs::PointField& field)
{
  switch (field.datatype)
  {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1U;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2U;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4U;
    case sensor_msgs::PointField::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

}  // namespace

class TemporalDynamicFilter
{
public:
  TemporalDynamicFilter()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/rslidar_points");
    private_nh_.param<std::string>("static_topic", static_topic_, "/rslidar_points_static");
    private_nh_.param<std::string>("dynamic_topic", dynamic_topic_, "/rslidar_points_dynamic_candidates");
    private_nh_.param("publish_dynamic_candidates", publish_dynamic_candidates_, true);
    private_nh_.param("azimuth_bin_deg", azimuth_bin_deg_, 0.4);
    private_nh_.param("elevation_bin_deg", elevation_bin_deg_, 0.1);
    private_nh_.param("range_tolerance_m", range_tolerance_m_, 0.20);
    private_nh_.param("range_tolerance_ratio", range_tolerance_ratio_, 0.005);
    private_nh_.param("range_ema_alpha", range_ema_alpha_, 0.25);
    private_nh_.param("stable_frames", stable_frames_, 3);
    private_nh_.param("foreground_persistence_frames", foreground_persistence_frames_, 30);
    private_nh_.param("clearing_persistence_frames", clearing_persistence_frames_, 3);
    private_nh_.param("state_timeout_frames", state_timeout_frames_, 30);
    private_nh_.param("fail_open", fail_open_, true);

    azimuth_bin_deg_ = std::max(0.01, azimuth_bin_deg_);
    elevation_bin_deg_ = std::max(0.01, elevation_bin_deg_);
    range_tolerance_m_ = std::max(0.01, range_tolerance_m_);
    range_tolerance_ratio_ = std::max(0.0, range_tolerance_ratio_);
    range_ema_alpha_ = std::max(0.0, std::min(1.0, range_ema_alpha_));
    stable_frames_ = std::max(1, stable_frames_);
    foreground_persistence_frames_ = std::max(stable_frames_, foreground_persistence_frames_);
    clearing_persistence_frames_ = std::max(1, clearing_persistence_frames_);
    state_timeout_frames_ = std::max(1, state_timeout_frames_);
    azimuth_bin_count_ = std::max(1, static_cast<int>(std::ceil(360.0 / azimuth_bin_deg_)));

    static_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_topic_, 2);
    if (publish_dynamic_candidates_)
    {
      dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_topic_, 2);
    }
    cloud_sub_ = nh_.subscribe(input_topic_, 2, &TemporalDynamicFilter::cloudCallback, this);

    ROS_INFO_STREAM("Temporal dynamic filter: " << input_topic_ << " -> " << static_topic_);
    ROS_INFO_STREAM("  bins azimuth/elevation: " << azimuth_bin_deg_ << "/" << elevation_bin_deg_
                    << " deg, stable frames: " << stable_frames_
                    << ", foreground persistence: " << foreground_persistence_frames_);
    ROS_WARN("This is a temporal background filter. Slow or stationary objects can eventually be classified as static.");
  }

private:
  const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                           const std::string& name) const
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

  bool validField(const sensor_msgs::PointCloud2& cloud,
                  const sensor_msgs::PointField* field,
                  std::size_t required_size) const
  {
    return field != nullptr && field->count >= 1U &&
           field->offset + required_size <= cloud.point_step;
  }

  bool readRing(const std::uint8_t* point_data,
                const sensor_msgs::PointField& field,
                int& ring) const
  {
    const std::uint8_t* data = point_data + field.offset;
    switch (field.datatype)
    {
      case sensor_msgs::PointField::UINT8:
        ring = static_cast<int>(readValue<std::uint8_t>(data));
        return true;
      case sensor_msgs::PointField::UINT16:
        ring = static_cast<int>(readValue<std::uint16_t>(data));
        return true;
      case sensor_msgs::PointField::UINT32:
      {
        const std::uint32_t value = readValue<std::uint32_t>(data);
        if (value > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
        {
          return false;
        }
        ring = static_cast<int>(value);
        return true;
      }
      default:
        return false;
    }
  }

  BeamKey makeKey(double x, double y, double z, int ring, bool has_ring) const
  {
    double azimuth_deg = std::atan2(y, x) * kRadToDeg;
    if (azimuth_deg < 0.0)
    {
      azimuth_deg += 360.0;
    }

    int azimuth_bin = static_cast<int>(std::floor(azimuth_deg / azimuth_bin_deg_));
    azimuth_bin = std::max(0, std::min(azimuth_bin_count_ - 1, azimuth_bin));

    int channel = ring;
    if (!has_ring)
    {
      const double horizontal_range = std::hypot(x, y);
      const double elevation_deg = std::atan2(z, horizontal_range) * kRadToDeg;
      channel = static_cast<int>(std::lround(elevation_deg / elevation_bin_deg_));
    }

    return BeamKey{channel, azimuth_bin};
  }

  double tolerance(double first_range, double second_range) const
  {
    return range_tolerance_m_ +
           range_tolerance_ratio_ * std::min(first_range, second_range);
  }

  bool rangesClose(double first_range, double second_range) const
  {
    return std::abs(first_range - second_range) <= tolerance(first_range, second_range);
  }

  bool updateState(const BeamKey& key, double measured_range)
  {
    auto inserted = states_.emplace(key, CellState{});
    CellState& state = inserted.first->second;

    if (inserted.second || frame_index_ - state.last_seen_frame >
                               static_cast<std::uint64_t>(state_timeout_frames_))
    {
      state.background_range = measured_range;
      state.stable_hits = 1;
      state.candidate_range = 0.0;
      state.candidate_hits = 0;
      state.last_seen_frame = frame_index_;
      return stable_frames_ <= 1;
    }

    state.last_seen_frame = frame_index_;

    if (rangesClose(measured_range, state.background_range))
    {
      state.background_range =
        (1.0 - range_ema_alpha_) * state.background_range + range_ema_alpha_ * measured_range;
      state.stable_hits = std::min(stable_frames_, state.stable_hits + 1);
      state.candidate_hits = 0;
      return state.stable_hits >= stable_frames_;
    }

    if (state.candidate_hits > 0 && rangesClose(measured_range, state.candidate_range))
    {
      state.candidate_range =
        (1.0 - range_ema_alpha_) * state.candidate_range + range_ema_alpha_ * measured_range;
      ++state.candidate_hits;
    }
    else
    {
      state.candidate_range = measured_range;
      state.candidate_hits = 1;
    }

    // A new, nearer return is commonly a person temporarily occluding the
    // static background. It must persist longer than a newly revealed,
    // farther background before replacing the model.
    const bool nearer_than_background = state.candidate_range < state.background_range;
    const int replacement_threshold = nearer_than_background ?
      foreground_persistence_frames_ : clearing_persistence_frames_;

    if (state.candidate_hits >= replacement_threshold)
    {
      state.background_range = state.candidate_range;
      state.stable_hits = stable_frames_;
      state.candidate_hits = 0;
      return true;
    }

    return false;
  }

  sensor_msgs::PointCloud2 buildCloud(const sensor_msgs::PointCloud2& input,
                                      const std::vector<std::size_t>& offsets) const
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

    std::size_t destination_offset = 0U;
    for (const std::size_t source_offset : offsets)
    {
      std::memcpy(output.data.data() + destination_offset,
                  input.data.data() + source_offset,
                  input.point_step);
      destination_offset += input.point_step;
    }
    return output;
  }

  void publishFailOpen(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& reason)
  {
    ROS_ERROR_THROTTLE(2.0, "Temporal dynamic filter bypassed: %s", reason.c_str());
    if (fail_open_)
    {
      static_pub_.publish(*cloud);
    }
  }

  void cleanupOldStates()
  {
    if (frame_index_ % 100U != 0U)
    {
      return;
    }

    for (auto it = states_.begin(); it != states_.end();)
    {
      if (frame_index_ - it->second.last_seen_frame >
          static_cast<std::uint64_t>(state_timeout_frames_))
      {
        it = states_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if (cloud->is_bigendian)
    {
      publishFailOpen(cloud, "big-endian PointCloud2 is not supported");
      return;
    }

    const sensor_msgs::PointField* x_field = findField(*cloud, "x");
    const sensor_msgs::PointField* y_field = findField(*cloud, "y");
    const sensor_msgs::PointField* z_field = findField(*cloud, "z");
    const sensor_msgs::PointField* ring_field = findField(*cloud, "ring");

    if (!validField(*cloud, x_field, sizeof(float)) ||
        !validField(*cloud, y_field, sizeof(float)) ||
        !validField(*cloud, z_field, sizeof(float)) ||
        x_field->datatype != sensor_msgs::PointField::FLOAT32 ||
        y_field->datatype != sensor_msgs::PointField::FLOAT32 ||
        z_field->datatype != sensor_msgs::PointField::FLOAT32)
    {
      publishFailOpen(cloud, "x/y/z FLOAT32 fields are required");
      return;
    }

    const bool has_ring = ring_field != nullptr &&
                          fieldSize(*ring_field) > 0U &&
                          validField(*cloud, ring_field, fieldSize(*ring_field));

    if (!last_stamp_.isZero() && cloud->header.stamp < last_stamp_)
    {
      ROS_WARN("Point cloud time moved backwards; resetting temporal background model.");
      states_.clear();
      frame_index_ = 0U;
    }
    last_stamp_ = cloud->header.stamp;
    ++frame_index_;

    std::vector<Observation> observations;
    observations.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
    std::unordered_map<BeamKey, double, BeamKeyHash> measurements;

    for (std::uint32_t row = 0U; row < cloud->height; ++row)
    {
      for (std::uint32_t col = 0U; col < cloud->width; ++col)
      {
        const std::size_t offset = static_cast<std::size_t>(row) * cloud->row_step +
                                   static_cast<std::size_t>(col) * cloud->point_step;
        if (offset + cloud->point_step > cloud->data.size())
        {
          publishFailOpen(cloud, "PointCloud2 data are shorter than width/height metadata");
          return;
        }

        const std::uint8_t* point_data = cloud->data.data() + offset;
        const double x = static_cast<double>(readValue<float>(point_data + x_field->offset));
        const double y = static_cast<double>(readValue<float>(point_data + y_field->offset));
        const double z = static_cast<double>(readValue<float>(point_data + z_field->offset));

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
          continue;
        }

        const double range = std::sqrt(x * x + y * y + z * z);
        if (!std::isfinite(range) || range <= 0.0)
        {
          continue;
        }

        int ring = 0;
        bool point_has_ring = has_ring;
        if (point_has_ring && !readRing(point_data, *ring_field, ring))
        {
          point_has_ring = false;
        }

        const BeamKey key = makeKey(x, y, z, ring, point_has_ring);
        observations.push_back(Observation{key, range, offset});

        const auto found = measurements.find(key);
        if (found == measurements.end() || range < found->second)
        {
          measurements[key] = range;
        }
      }
    }

    std::unordered_map<BeamKey, bool, BeamKeyHash> stable_measurements;
    stable_measurements.reserve(measurements.size());
    for (const auto& measurement : measurements)
    {
      stable_measurements.emplace(measurement.first,
                                  updateState(measurement.first, measurement.second));
    }

    std::vector<std::size_t> static_offsets;
    std::vector<std::size_t> dynamic_offsets;
    static_offsets.reserve(observations.size());
    dynamic_offsets.reserve(observations.size() / 8U);

    for (const Observation& observation : observations)
    {
      const auto stable = stable_measurements.find(observation.key);
      const auto measured = measurements.find(observation.key);
      const bool is_static = stable != stable_measurements.end() && stable->second &&
                             measured != measurements.end() &&
                             rangesClose(observation.range, measured->second);
      if (is_static)
      {
        static_offsets.push_back(observation.data_offset);
      }
      else
      {
        dynamic_offsets.push_back(observation.data_offset);
      }
    }

    static_pub_.publish(buildCloud(*cloud, static_offsets));
    if (publish_dynamic_candidates_)
    {
      dynamic_pub_.publish(buildCloud(*cloud, dynamic_offsets));
    }

    cleanupOldStates();
    ROS_INFO_THROTTLE(2.0,
                      "Temporal cloud filter: input=%zu static=%zu candidates=%zu model_cells=%zu",
                      observations.size(), static_offsets.size(), dynamic_offsets.size(), states_.size());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher static_pub_;
  ros::Publisher dynamic_pub_;

  std::string input_topic_;
  std::string static_topic_;
  std::string dynamic_topic_;
  bool publish_dynamic_candidates_ = true;
  bool fail_open_ = true;
  double azimuth_bin_deg_ = 0.4;
  double elevation_bin_deg_ = 0.1;
  double range_tolerance_m_ = 0.20;
  double range_tolerance_ratio_ = 0.005;
  double range_ema_alpha_ = 0.25;
  int stable_frames_ = 3;
  int foreground_persistence_frames_ = 30;
  int clearing_persistence_frames_ = 3;
  int state_timeout_frames_ = 30;
  int azimuth_bin_count_ = 900;

  std::uint64_t frame_index_ = 0U;
  ros::Time last_stamp_;
  std::unordered_map<BeamKey, CellState, BeamKeyHash> states_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temporal_dynamic_filter");
  TemporalDynamicFilter filter;
  ros::spin();
  return 0;
}
