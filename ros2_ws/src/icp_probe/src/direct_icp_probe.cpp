#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double YawFromTransform(const Eigen::Matrix4f &transform) {
  return std::atan2(transform(1, 0), transform(0, 0));
}

Eigen::Affine3f TransformToEigen(
    const geometry_msgs::msg::TransformStamped &transform) {
  const auto &translation = transform.transform.translation;
  const auto &rotation = transform.transform.rotation;
  Eigen::Quaternionf quaternion(
      static_cast<float>(rotation.w), static_cast<float>(rotation.x),
      static_cast<float>(rotation.y), static_cast<float>(rotation.z));
  if (quaternion.norm() < 1e-6F) {
    quaternion = Eigen::Quaternionf::Identity();
  } else {
    quaternion.normalize();
  }

  Eigen::Affine3f result = Eigen::Affine3f::Identity();
  result.translation() = Eigen::Vector3f(
      static_cast<float>(translation.x), static_cast<float>(translation.y),
      static_cast<float>(translation.z));
  result.linear() = quaternion.toRotationMatrix();
  return result;
}

Eigen::Affine3f PoseToEigen(const geometry_msgs::msg::Pose &pose) {
  Eigen::Quaternionf quaternion(
      static_cast<float>(pose.orientation.w),
      static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y),
      static_cast<float>(pose.orientation.z));
  if (quaternion.norm() < 1e-6F) {
    quaternion = Eigen::Quaternionf::Identity();
  } else {
    quaternion.normalize();
  }

  Eigen::Affine3f result = Eigen::Affine3f::Identity();
  result.translation() = Eigen::Vector3f(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z));
  result.linear() = quaternion.toRotationMatrix();
  return result;
}

}  // namespace

class DirectIcpProbe : public rclcpp::Node {
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  struct MatchMetrics {
    std::size_t inliers = 0;
    double overlap = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
  };

  DirectIcpProbe()
      : Node("direct_icp_probe"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_),
        tf_broadcaster_(*this) {
    const auto map_path =
        declare_parameter<std::string>("map_path", "");
    input_topic_ =
        declare_parameter<std::string>("input_topic", "/livox/lidar");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    initial_pose_frame_ =
        declare_parameter<std::string>("initial_pose_frame", "base_link");
    initial_pose_topic_ =
        declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    publish_map_to_odom_ =
        declare_parameter<bool>("publish_map_to_odom", true);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    const auto tf_publish_rate =
        declare_parameter<double>("tf_publish_rate", 10.0);
    frames_to_accumulate_ =
        declare_parameter<int>("frames_to_accumulate", 5);
    min_range_ = declare_parameter<double>("min_range", 0.50);
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.25);
    rough_voxel_leaf_ =
        declare_parameter<double>("rough_voxel_leaf", 0.60);
    max_range_ = declare_parameter<double>("max_range", 35.0);
    target_crop_radius_ =
        declare_parameter<double>("target_crop_radius", 12.0);
    max_correspondence_ =
        declare_parameter<double>("max_correspondence", 2.0);
    rough_max_correspondence_ =
        declare_parameter<double>("rough_max_correspondence", 3.0);
    max_iterations_ = declare_parameter<int>("max_iterations", 80);
    rough_iterations_ = declare_parameter<int>("rough_iterations", 15);
    fitness_threshold_ =
        declare_parameter<double>("fitness_threshold", 0.80);
    map_min_z_ = declare_parameter<double>("map_min_z", 0.20);
    map_max_z_ = declare_parameter<double>("map_max_z", 1.50);
    min_overlap_ratio_ =
        declare_parameter<double>("min_overlap_ratio", 0.35);
    max_inlier_rmse_ =
        declare_parameter<double>("max_inlier_rmse", 0.40);
    max_seed_xy_correction_ =
        declare_parameter<double>("max_seed_xy_correction", 0.80);
    max_seed_z_correction_ =
        declare_parameter<double>("max_seed_z_correction", 0.40);
    max_seed_yaw_correction_deg_ = declare_parameter<double>(
        "max_seed_yaw_correction_deg", 35.0);
    retry_on_failure_ =
        declare_parameter<bool>("retry_on_failure", true);
    retry_skip_frames_ =
        declare_parameter<int>("retry_skip_frames", 15);
    required_consistent_results_ =
        declare_parameter<int>("required_consistent_results", 2);
    consistency_translation_ =
        declare_parameter<double>("consistency_translation", 0.25);
    consistency_yaw_deg_ =
        declare_parameter<double>("consistency_yaw_deg", 5.0);
    xy_offset_ = declare_parameter<double>("xy_offset", 0.50);
    xy_search_steps_ = declare_parameter<int>("xy_search_steps", 0);
    yaw_offset_deg_ = declare_parameter<double>("yaw_offset_deg", 30.0);
    yaw_resolution_deg_ =
        declare_parameter<double>("yaw_resolution_deg", 15.0);

    const auto initial_x = declare_parameter<double>("initial_x", 0.0);
    const auto initial_y = declare_parameter<double>("initial_y", 0.0);
    const auto initial_z = declare_parameter<double>("initial_z", 0.0);
    const auto initial_yaw = declare_parameter<double>("initial_yaw", 0.0);
    seed_map_to_initial_frame_ = Eigen::Affine3f::Identity();
    seed_map_to_initial_frame_.translation() = Eigen::Vector3f(
        static_cast<float>(initial_x), static_cast<float>(initial_y),
        static_cast<float>(initial_z));
    seed_map_to_initial_frame_.rotate(Eigen::AngleAxisf(
        static_cast<float>(initial_yaw), Eigen::Vector3f::UnitZ()));

    ValidateParameters();

    Cloud::Ptr raw_map(new Cloud);
    if (pcl::io::loadPCDFile<Point>(map_path, *raw_map) != 0) {
      throw std::runtime_error("Failed to load PCD map: " + map_path);
    }
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*raw_map, *raw_map, finite_indices);
    auto height_filtered_map = FilterMapHeight(raw_map);
    target_ = Downsample(height_filtered_map, voxel_leaf_);
    RCLCPP_INFO(get_logger(),
                "Loaded map %s: %zu finite -> %zu height-filtered -> %zu "
                "voxel points",
                map_path.c_str(), raw_map->size(), height_filtered_map->size(),
                target_->size());

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/icp_probe/pose", rclcpp::QoS(1).transient_local());
    aligned_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/icp_probe/aligned", rclcpp::QoS(1).transient_local());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&DirectIcpProbe::CloudCallback, this,
                  std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, rclcpp::QoS(10),
        std::bind(&DirectIcpProbe::InitialPoseCallback, this,
                  std::placeholders::_1));
    tf_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(tf_publish_rate, 0.1)),
        std::bind(&DirectIcpProbe::PublishMapToOdom, this));

    capture_enabled_ = auto_start_;
    RCLCPP_INFO(
        get_logger(),
        "ICP ready: input=%s frames=%d auto_start=%s; publish a 2D Pose "
        "Estimate on %s to seed or repeat localization; %d consistent "
        "result(s) required",
        input_topic_.c_str(), frames_to_accumulate_,
        auto_start_ ? "true" : "false", initial_pose_topic_.c_str(),
        required_consistent_results_);
  }

 private:
  void ValidateParameters() const {
    if (frames_to_accumulate_ <= 0) {
      throw std::invalid_argument("frames_to_accumulate must be positive");
    }
    if (voxel_leaf_ <= 0.0 || rough_voxel_leaf_ <= 0.0) {
      throw std::invalid_argument("voxel leaf sizes must be positive");
    }
    if (min_range_ < 0.0 || max_range_ <= min_range_ ||
        max_correspondence_ <= 0.0 ||
        rough_max_correspondence_ <= 0.0) {
      throw std::invalid_argument(
          "range and correspondence distances must be positive");
    }
    if (max_iterations_ <= 0 || rough_iterations_ <= 0) {
      throw std::invalid_argument("ICP iteration counts must be positive");
    }
    if (xy_search_steps_ < 0 || xy_search_steps_ > 10) {
      throw std::invalid_argument("xy_search_steps must be between 0 and 10");
    }
    if (yaw_offset_deg_ < 0.0 || yaw_offset_deg_ > 180.0 ||
        yaw_resolution_deg_ <= 0.0) {
      throw std::invalid_argument(
          "yaw search requires 0 <= yaw_offset_deg <= 180 and a positive "
          "yaw_resolution_deg");
    }
    if (map_min_z_ >= map_max_z_) {
      throw std::invalid_argument("map_min_z must be less than map_max_z");
    }
    if (min_overlap_ratio_ < 0.0 || min_overlap_ratio_ > 1.0 ||
        max_inlier_rmse_ <= 0.0) {
      throw std::invalid_argument(
          "overlap must be within [0, 1] and RMSE must be positive");
    }
    if (max_seed_xy_correction_ <= 0.0 ||
        max_seed_z_correction_ <= 0.0 ||
        max_seed_yaw_correction_deg_ <= 0.0 ||
        max_seed_yaw_correction_deg_ > 180.0) {
      throw std::invalid_argument("seed correction limits are invalid");
    }
    if (retry_skip_frames_ < 0 || required_consistent_results_ <= 0 ||
        consistency_translation_ <= 0.0 || consistency_yaw_deg_ <= 0.0) {
      throw std::invalid_argument("retry and consistency parameters are invalid");
    }
  }

  Cloud::Ptr FilterMapHeight(const Cloud::ConstPtr &input) const {
    Cloud::Ptr output(new Cloud);
    output->reserve(input->size());
    for (const auto &point : *input) {
      if (point.z >= map_min_z_ && point.z <= map_max_z_) {
        output->push_back(point);
      }
    }
    return output;
  }

  Cloud::Ptr FilterSourceHeight(
      const Cloud::ConstPtr &input,
      const Eigen::Affine3f &map_to_source_guess) const {
    Cloud::Ptr output(new Cloud);
    output->reserve(input->size());
    for (const auto &point : *input) {
      const Eigen::Vector3f map_point =
          map_to_source_guess * Eigen::Vector3f(point.x, point.y, point.z);
      if (map_point.z() >= map_min_z_ && map_point.z() <= map_max_z_) {
        output->push_back(point);
      }
    }
    return output;
  }

  Cloud::Ptr Downsample(const Cloud::ConstPtr &input, double leaf) const {
    pcl::VoxelGrid<Point> voxel;
    const auto leaf_float = static_cast<float>(leaf);
    voxel.setLeafSize(leaf_float, leaf_float, leaf_float);
    voxel.setInputCloud(input);
    Cloud::Ptr output(new Cloud);
    voxel.filter(*output);
    return output;
  }

  Cloud::Ptr CropTarget(const Eigen::Vector3f &center) const {
    if (target_crop_radius_ <= 0.0) {
      return Cloud::Ptr(new Cloud(*target_));
    }

    const float radius_squared = static_cast<float>(
        target_crop_radius_ * target_crop_radius_);
    Cloud::Ptr output(new Cloud);
    output->reserve(target_->size());
    for (const auto &point : *target_) {
      const float dx = point.x - center.x();
      const float dy = point.y - center.y();
      if (dx * dx + dy * dy <= radius_squared) {
        output->push_back(point);
      }
    }
    if (output->size() < 100) {
      RCLCPP_WARN(get_logger(),
                  "Target crop contains only %zu points; using the full map",
                  output->size());
      return Cloud::Ptr(new Cloud(*target_));
    }
    return output;
  }

  void InitialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    if (!message->header.frame_id.empty() &&
        message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(get_logger(),
                   "Ignoring initial pose in frame '%s'; expected '%s'",
                   message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    seed_map_to_initial_frame_ = PoseToEigen(message->pose.pose);
    accumulated_->clear();
    received_frames_ = 0;
    source_frame_.clear();
    retry_frames_remaining_ = 0;
    consistent_results_ = 0;
    capture_enabled_ = true;
    RCLCPP_INFO(get_logger(),
                "Initial pose received; collecting %d fresh frames for ICP",
                frames_to_accumulate_);
  }

  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (!capture_enabled_ || registration_running_) {
      return;
    }
    if (retry_frames_remaining_ > 0) {
      --retry_frames_remaining_;
      return;
    }
    if (source_frame_.empty()) {
      source_frame_ = message->header.frame_id;
    } else if (source_frame_ != message->header.frame_id) {
      RCLCPP_ERROR(get_logger(),
                   "Input frame changed from '%s' to '%s'; restarting capture",
                   source_frame_.c_str(), message->header.frame_id.c_str());
      accumulated_->clear();
      received_frames_ = 0;
      source_frame_ = message->header.frame_id;
    }

    Cloud frame;
    pcl::fromROSMsg(*message, frame);
    const float min_range_squared =
        static_cast<float>(min_range_ * min_range_);
    const float max_range_squared =
        static_cast<float>(max_range_ * max_range_);
    for (const auto &point : frame) {
      const float range_squared =
          point.x * point.x + point.y * point.y + point.z * point.z;
      if (pcl::isFinite(point) && range_squared >= min_range_squared &&
          range_squared <= max_range_squared) {
        accumulated_->push_back(point);
      }
    }
    ++received_frames_;
    RCLCPP_INFO(get_logger(), "Accumulated frame %d/%d (%zu points)",
                received_frames_, frames_to_accumulate_, accumulated_->size());
    if (received_frames_ < frames_to_accumulate_) {
      return;
    }

    capture_enabled_ = false;
    registration_running_ = true;
    try {
      odom_to_source_ = tf_buffer_.lookupTransform(
          odom_frame_, source_frame_, tf2::TimePointZero);
      have_odom_to_source_ = true;
      RCLCPP_INFO(get_logger(), "Captured %s->%s TF before registration",
                  odom_frame_.c_str(), source_frame_.c_str());
    } catch (const tf2::TransformException &error) {
      have_odom_to_source_ = false;
      registration_running_ = false;
      accumulated_->clear();
      received_frames_ = 0;
      capture_enabled_ = true;
      RCLCPP_ERROR(get_logger(),
                   "Cannot localize: failed to look up %s->%s: %s",
                   odom_frame_.c_str(), source_frame_.c_str(), error.what());
      return;
    }

    const bool localized = RunRegistration(message->header.stamp);
    registration_running_ = false;
    if (!localized && retry_on_failure_) {
      PrepareRetry();
    }
  }

  void PrepareRetry() {
    accumulated_->clear();
    received_frames_ = 0;
    source_frame_.clear();
    retry_frames_remaining_ = retry_skip_frames_;
    capture_enabled_ = true;
    RCLCPP_WARN(get_logger(),
                "Localization not accepted; retrying with fresh scans after "
                "%d skipped frame(s)",
                retry_skip_frames_);
  }

  Eigen::Affine3f InitialMapToSourceGuess() {
    if (initial_pose_frame_ == source_frame_) {
      return seed_map_to_initial_frame_;
    }
    try {
      const auto initial_to_source = tf_buffer_.lookupTransform(
          initial_pose_frame_, source_frame_, tf2::TimePointZero);
      return seed_map_to_initial_frame_ * TransformToEigen(initial_to_source);
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN(
          get_logger(),
          "Could not transform initial-pose frame %s to cloud frame %s: %s. "
          "Treating the seed as a cloud-frame pose.",
          initial_pose_frame_.c_str(), source_frame_.c_str(), error.what());
      return seed_map_to_initial_frame_;
    }
  }

  std::vector<Eigen::Matrix4f> BuildCandidates(
      const Eigen::Affine3f &base_guess) const {
    const int yaw_steps = static_cast<int>(
        std::floor(yaw_offset_deg_ / yaw_resolution_deg_ + 1e-9));
    const double yaw_resolution_rad = yaw_resolution_deg_ * kPi / 180.0;
    std::vector<Eigen::Matrix4f> candidates;
    candidates.reserve(
        static_cast<std::size_t>((2 * xy_search_steps_ + 1) *
                                 (2 * xy_search_steps_ + 1) *
                                 (2 * yaw_steps + 1)));

    for (int x_step = -xy_search_steps_; x_step <= xy_search_steps_;
         ++x_step) {
      for (int y_step = -xy_search_steps_; y_step <= xy_search_steps_;
           ++y_step) {
        for (int yaw_step = -yaw_steps; yaw_step <= yaw_steps; ++yaw_step) {
          Eigen::Affine3f candidate = base_guess;
          candidate.translation().x() +=
              static_cast<float>(x_step * xy_offset_);
          candidate.translation().y() +=
              static_cast<float>(y_step * xy_offset_);
          const Eigen::AngleAxisf yaw_delta(
              static_cast<float>(yaw_step * yaw_resolution_rad),
              Eigen::Vector3f::UnitZ());
          candidate.linear() = yaw_delta.toRotationMatrix() *
                               base_guess.linear();
          candidates.push_back(candidate.matrix());
        }
      }
    }
    return candidates;
  }

  Eigen::Matrix4f RunRoughSearch(
      const Cloud::ConstPtr &source, const Cloud::ConstPtr &target,
      const std::vector<Eigen::Matrix4f> &candidates) {
    if (candidates.size() == 1) {
      return candidates.front();
    }

    auto rough_source = Downsample(source, rough_voxel_leaf_);
    auto rough_target = Downsample(target, rough_voxel_leaf_);
    double best_fitness = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f best_transform = candidates.front();
    std::size_t converged_count = 0;

    pcl::IterativeClosestPoint<Point, Point> registration;
    registration.setInputSource(rough_source);
    registration.setInputTarget(rough_target);
    registration.setMaximumIterations(rough_iterations_);
    registration.setMaxCorrespondenceDistance(rough_max_correspondence_);
    registration.setTransformationEpsilon(1e-3);
    registration.setEuclideanFitnessEpsilon(1e-3);

    Cloud aligned;
    for (const auto &candidate : candidates) {
      registration.align(aligned, candidate);
      if (!registration.hasConverged()) {
        continue;
      }
      ++converged_count;
      const double fitness = registration.getFitnessScore();
      if (fitness < best_fitness) {
        best_fitness = fitness;
        best_transform = registration.getFinalTransformation();
      }
    }

    RCLCPP_INFO(get_logger(),
                "Rough search: %zu candidates, %zu converged, best fitness=%.6f",
                candidates.size(), converged_count, best_fitness);
    return best_transform;
  }

  MatchMetrics MeasureAlignment(const Cloud::ConstPtr &aligned,
                                const Cloud::ConstPtr &target) const {
    MatchMetrics metrics;
    if (aligned->empty() || target->empty()) {
      return metrics;
    }

    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_distance_squared(1);
    const float maximum_distance_squared = static_cast<float>(
        max_correspondence_ * max_correspondence_);
    double squared_error_sum = 0.0;
    for (const auto &point : *aligned) {
      if (tree.nearestKSearch(point, 1, nearest_index,
                              nearest_distance_squared) > 0 &&
          nearest_distance_squared.front() <= maximum_distance_squared) {
        ++metrics.inliers;
        squared_error_sum += nearest_distance_squared.front();
      }
    }

    metrics.overlap = static_cast<double>(metrics.inliers) /
                      static_cast<double>(aligned->size());
    if (metrics.inliers > 0) {
      metrics.rmse =
          std::sqrt(squared_error_sum / static_cast<double>(metrics.inliers));
    }
    return metrics;
  }

  bool IsPlausibleFromSeed(const Eigen::Matrix4f &base_guess,
                           const Eigen::Matrix4f &candidate) const {
    const double dx = candidate(0, 3) - base_guess(0, 3);
    const double dy = candidate(1, 3) - base_guess(1, 3);
    const double dz = candidate(2, 3) - base_guess(2, 3);
    const double xy_correction = std::hypot(dx, dy);
    const double yaw_correction = std::abs(NormalizeAngle(
        YawFromTransform(candidate) - YawFromTransform(base_guess)));
    const double yaw_limit = max_seed_yaw_correction_deg_ * kPi / 180.0;

    RCLCPP_INFO(get_logger(),
                "Correction from seed: xy=%.3f m z=%.3f m yaw=%.2f deg",
                xy_correction, std::abs(dz), yaw_correction * 180.0 / kPi);
    if (xy_correction > max_seed_xy_correction_ ||
        std::abs(dz) > max_seed_z_correction_ ||
        yaw_correction > yaw_limit) {
      RCLCPP_ERROR(
          get_logger(),
          "Registration rejected: correction exceeds limits "
          "(xy<=%.2f m, z<=%.2f m, yaw<=%.1f deg)",
          max_seed_xy_correction_, max_seed_z_correction_,
          max_seed_yaw_correction_deg_);
      return false;
    }
    return true;
  }

  bool IsConsistent(const Eigen::Affine3f &first,
                    const Eigen::Affine3f &second) const {
    const double dx = second.translation().x() - first.translation().x();
    const double dy = second.translation().y() - first.translation().y();
    const double translation = std::hypot(dx, dy);
    const double yaw_delta = std::abs(NormalizeAngle(
        YawFromTransform(second.matrix()) - YawFromTransform(first.matrix())));
    RCLCPP_INFO(get_logger(),
                "Consistency delta: translation=%.3f m yaw=%.2f deg",
                translation, yaw_delta * 180.0 / kPi);
    return translation <= consistency_translation_ &&
           yaw_delta <= consistency_yaw_deg_ * kPi / 180.0;
  }

  bool RunRegistration(const builtin_interfaces::msg::Time &stamp) {
    const Eigen::Affine3f base_guess = InitialMapToSourceGuess();
    auto height_filtered_source =
        FilterSourceHeight(accumulated_, base_guess);
    auto source = Downsample(height_filtered_source, voxel_leaf_);
    RCLCPP_INFO(get_logger(),
                "Source filtered: %zu -> %zu height-filtered -> %zu voxel "
                "points",
                accumulated_->size(), height_filtered_source->size(),
                source->size());
    if (source->size() < 100 || target_->size() < 100) {
      RCLCPP_ERROR(get_logger(), "Insufficient points for registration");
      consistent_results_ = 0;
      return false;
    }

    auto local_target = CropTarget(base_guess.translation());
    const auto candidates = BuildCandidates(base_guess);
    RCLCPP_INFO(
        get_logger(),
        "Registration seed: x=%.3f y=%.3f z=%.3f; target=%zu points; "
        "candidates=%zu",
        base_guess.translation().x(), base_guess.translation().y(),
        base_guess.translation().z(), local_target->size(), candidates.size());

    const auto start = now();
    const Eigen::Matrix4f rough_guess =
        RunRoughSearch(source, local_target, candidates);

    pcl::GeneralizedIterativeClosestPoint<Point, Point> registration;
    registration.setInputSource(source);
    registration.setInputTarget(local_target);
    registration.setMaximumIterations(max_iterations_);
    registration.setMaxCorrespondenceDistance(max_correspondence_);
    registration.setTransformationEpsilon(1e-4);
    registration.setEuclideanFitnessEpsilon(1e-4);

    Cloud aligned;
    registration.align(aligned, rough_guess);
    const auto elapsed = (now() - start).seconds();
    const double fitness = registration.getFitnessScore();
    const Eigen::Matrix4f map_to_source =
        registration.getFinalTransformation();
    const double yaw = YawFromTransform(map_to_source);
    const Cloud::ConstPtr aligned_cloud(new Cloud(aligned));
    const MatchMetrics metrics =
        MeasureAlignment(aligned_cloud, local_target);

    RCLCPP_INFO(get_logger(),
                "GICP result: converged=%s fitness=%.6f overlap=%.1f%% "
                "inlier_rmse=%.4f m (%zu/%zu) elapsed=%.3fs",
                registration.hasConverged() ? "true" : "false", fitness,
                metrics.overlap * 100.0, metrics.rmse, metrics.inliers,
                aligned.size(), elapsed);
    RCLCPP_INFO(get_logger(),
                "Candidate %s->%s: x=%.4f y=%.4f z=%.4f yaw=%.4f rad",
                map_frame_.c_str(), source_frame_.c_str(), map_to_source(0, 3),
                map_to_source(1, 3), map_to_source(2, 3), yaw);
    if (!registration.hasConverged()) {
      RCLCPP_ERROR(get_logger(),
                   "Registration did not converge; keeping the previous TF");
      consistent_results_ = 0;
      return false;
    }
    if (fitness_threshold_ > 0.0 && fitness > fitness_threshold_) {
      RCLCPP_ERROR(
          get_logger(),
          "Registration rejected: fitness %.6f exceeds threshold %.6f. "
          "Give a closer 2D Pose Estimate and try again.",
          fitness, fitness_threshold_);
      consistent_results_ = 0;
      return false;
    }
    if (metrics.overlap < min_overlap_ratio_) {
      RCLCPP_ERROR(get_logger(),
                   "Registration rejected: overlap %.1f%% is below %.1f%%",
                   metrics.overlap * 100.0, min_overlap_ratio_ * 100.0);
      consistent_results_ = 0;
      return false;
    }
    if (!std::isfinite(metrics.rmse) || metrics.rmse > max_inlier_rmse_) {
      RCLCPP_ERROR(get_logger(),
                   "Registration rejected: inlier RMSE %.4f m exceeds %.4f m",
                   metrics.rmse, max_inlier_rmse_);
      consistent_results_ = 0;
      return false;
    }
    if (!IsPlausibleFromSeed(base_guess.matrix(), map_to_source)) {
      consistent_results_ = 0;
      return false;
    }

    sensor_msgs::msg::PointCloud2 aligned_message;
    pcl::toROSMsg(aligned, aligned_message);
    aligned_message.header.stamp = stamp;
    aligned_message.header.frame_id = map_frame_;
    aligned_publisher_->publish(aligned_message);

    if (!have_odom_to_source_) {
      consistent_results_ = 0;
      return false;
    }
    const Eigen::Affine3f candidate_map_to_odom =
        Eigen::Affine3f(map_to_source) *
        TransformToEigen(odom_to_source_).inverse();

    if (consistent_results_ == 0 ||
        !IsConsistent(pending_map_to_odom_, candidate_map_to_odom)) {
      pending_map_to_odom_ = candidate_map_to_odom;
      consistent_results_ = 1;
    } else {
      pending_map_to_odom_ = candidate_map_to_odom;
      ++consistent_results_;
    }
    if (consistent_results_ < required_consistent_results_) {
      RCLCPP_WARN(get_logger(),
                  "Candidate passed quality gates (%d/%d consistent); "
                  "collecting fresh scans before publishing TF",
                  consistent_results_, required_consistent_results_);
      return false;
    }

    const Eigen::Affine3f &map_to_odom = pending_map_to_odom_;

    map_to_odom_.header.frame_id = map_frame_;
    map_to_odom_.child_frame_id = odom_frame_;
    map_to_odom_.transform.translation.x = map_to_odom.translation().x();
    map_to_odom_.transform.translation.y = map_to_odom.translation().y();
    map_to_odom_.transform.translation.z = map_to_odom.translation().z();
    Eigen::Quaternionf map_odom_rotation(map_to_odom.rotation());
    map_odom_rotation.normalize();
    map_to_odom_.transform.rotation.x = map_odom_rotation.x();
    map_to_odom_.transform.rotation.y = map_odom_rotation.y();
    map_to_odom_.transform.rotation.z = map_odom_rotation.z();
    map_to_odom_.transform.rotation.w = map_odom_rotation.w();
    have_map_to_odom_ = true;
    PublishMapToOdom();

    PublishBasePose(map_to_source, stamp);
    if (publish_map_to_odom_) {
      RCLCPP_WARN(
          get_logger(),
          "Accepted localization (fitness %.6f, overlap %.1f%%, RMSE %.4f m) "
          "and publishing authoritative %s->%s TF; do not run AMCL or "
          "another map->odom publisher",
          fitness, metrics.overlap * 100.0, metrics.rmse, map_frame_.c_str(),
          odom_frame_.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "Accepted diagnostic localization without publishing TF");
    }
    return true;
  }

  void PublishBasePose(const Eigen::Matrix4f &map_to_source,
                       const builtin_interfaces::msg::Time &stamp) {
    Eigen::Affine3f map_to_initial_frame(map_to_source);
    if (initial_pose_frame_ != source_frame_) {
      try {
        const auto initial_to_source = tf_buffer_.lookupTransform(
            initial_pose_frame_, source_frame_, tf2::TimePointZero);
        map_to_initial_frame = Eigen::Affine3f(map_to_source) *
                               TransformToEigen(initial_to_source).inverse();
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN(get_logger(), "Publishing cloud-frame pose: %s",
                    error.what());
      }
    }

    Eigen::Quaternionf quaternion(map_to_initial_frame.rotation());
    quaternion.normalize();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = map_to_initial_frame.translation().x();
    pose.pose.position.y = map_to_initial_frame.translation().y();
    pose.pose.position.z = map_to_initial_frame.translation().z();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    pose_publisher_->publish(pose);
  }

  void PublishMapToOdom() {
    if (!publish_map_to_odom_ || !have_map_to_odom_) {
      return;
    }
    map_to_odom_.header.stamp = now();
    tf_broadcaster_.sendTransform(map_to_odom_);
  }

  std::string input_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string initial_pose_frame_;
  std::string initial_pose_topic_;
  std::string source_frame_;
  int frames_to_accumulate_;
  int received_frames_ = 0;
  int max_iterations_;
  int rough_iterations_;
  int xy_search_steps_;
  int retry_skip_frames_;
  int retry_frames_remaining_ = 0;
  int required_consistent_results_;
  int consistent_results_ = 0;
  double min_range_;
  double voxel_leaf_;
  double rough_voxel_leaf_;
  double max_range_;
  double target_crop_radius_;
  double max_correspondence_;
  double rough_max_correspondence_;
  double fitness_threshold_;
  double map_min_z_;
  double map_max_z_;
  double min_overlap_ratio_;
  double max_inlier_rmse_;
  double max_seed_xy_correction_;
  double max_seed_z_correction_;
  double max_seed_yaw_correction_deg_;
  double consistency_translation_;
  double consistency_yaw_deg_;
  double xy_offset_;
  double yaw_offset_deg_;
  double yaw_resolution_deg_;
  bool publish_map_to_odom_;
  bool auto_start_;
  bool retry_on_failure_;
  bool capture_enabled_ = false;
  bool registration_running_ = false;
  bool have_odom_to_source_ = false;
  bool have_map_to_odom_ = false;
  Cloud::Ptr target_;
  Cloud::Ptr accumulated_{new Cloud};
  Eigen::Affine3f seed_map_to_initial_frame_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f pending_map_to_odom_ = Eigen::Affine3f::Identity();
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_subscription_;
  rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  geometry_msgs::msg::TransformStamped odom_to_source_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DirectIcpProbe>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("direct_icp_probe"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
