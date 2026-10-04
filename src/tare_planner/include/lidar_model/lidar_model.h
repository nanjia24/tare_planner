/**
 * @file lidar_model.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the sensor model of a LiDAR
 * @version 0.1
 * @date 2019-09-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Geometry>
// ROS
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/misc_utils.h>

namespace lidar_model_ns
{
class LiDARModel
{
public:
  static double pointcloud_resolution_;
  explicit LiDARModel(double px = 0.0, double py = 0.0, double pz = 0.0, double rw = 1.0, double rx = 0.0,
                      double ry = 0.0, double rz = 0.0);
  explicit LiDARModel(const geometry_msgs::msg::Pose& pose);
  ~LiDARModel() = default;

  /**
   * @brief
   * TODO
   * @tparam PointType
   * @param point
   */
  template <class PointType>
  void UpdateCoverage(const PointType& point)
  {
    double distance_to_point = misc_utils_ns::PointXYZDist<PointType, geometry_msgs::msg::Point>(point, pose_.position);

    if (isZero(distance_to_point))
      return;

    double dx = point.x - pose_.position.x;
    double dy = point.y - pose_.position.y;
    double dz = point.z - pose_.position.z;

    if (directional_ && !InSensorFOV(point))
      return;

    int horizontal_angle = GetHorizontalAngle(dx, dy);
    int vertical_angle = GetVerticalAngle(dz, distance_to_point);

    int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
    int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

    for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; n++)
    {
      int column_index = horizontal_angle + n;
      if (directional_ || prediction_) column_index = (column_index % kHorizontalVoxelSize + kHorizontalVoxelSize) % kHorizontalVoxelSize;
      if (!ColumnIndexInRange(column_index))
        continue;
      for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; m++)
      {
        int row_index = vertical_angle + m;
        if (!RowIndexInRange(row_index))
          continue;
        int ind = sub2ind(row_index, column_index);
        double previous_distance_to_point = covered_voxel_[ind];
        if (isZero(previous_distance_to_point) || distance_to_point < previous_distance_to_point || reset_[ind])
        {
          covered_voxel_[ind] = distance_to_point;
          reset_[ind] = false;
        }
      }
    }
  }

  /**
   * @brief
   * TODO
   * @tparam PointType
   * @param point
   * @param occlusion_threshold
   * @return true
   * @return false
   */
  template <class PointType>
  bool CheckVisibility(const PointType& point, double occlusion_threshold) const
  {
    double distance_to_point = misc_utils_ns::PointXYZDist<PointType, geometry_msgs::msg::Point>(point, pose_.position);

    if (isZero(distance_to_point))
      return false;

    double dx = point.x - pose_.position.x;
    double dy = point.y - pose_.position.y;
    double dz = point.z - pose_.position.z;

    if (directional_ && !InSensorFOV(point))
      return false;

    int horizontal_angle = GetHorizontalAngle(dx, dy);
    int vertical_angle = GetVerticalAngle(dz, distance_to_point);

    int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
    int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

    for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; n++)
    {
      int column_index = horizontal_angle + n;
      if (directional_ || prediction_) column_index = (column_index % kHorizontalVoxelSize + kHorizontalVoxelSize) % kHorizontalVoxelSize;
      if (!ColumnIndexInRange(column_index))
        continue;
      for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; m++)
      {
        int row_index = vertical_angle + m;
        if (!RowIndexInRange(row_index))
          continue;
        int ind = sub2ind(row_index, column_index);
        float previous_distance_to_point = covered_voxel_[ind];
        if ((!isZero(previous_distance_to_point) &&
             distance_to_point < previous_distance_to_point + occlusion_threshold && !reset_[ind]) ||
            (!directional_ && reset_[ind]))
        {
          return true;
        }
      }
    }
    return false;
  }
  /**
   * @brief
   * TODO
   */
  void ResetCoverage();

  // Candidate occlusion cache is azimuth-independent: changing the estimated
  // arrival heading must not discard obstacles on the other side. The manager
  // gates each gain query by the forward FOV. Empty bins mean potential gain,
  // NOT measured free space, unlike ConfigureForwardSensor below.
  void ConfigurePrediction(double vertical_deg)
  {
    if (!std::isfinite(vertical_deg) || vertical_deg <= 0 || vertical_deg >= 180)
      throw std::invalid_argument("Invalid prediction vertical FOV");
    directional_ = false;
    prediction_ = true;
    const int half_bins = static_cast<int>(std::ceil(vertical_deg / (2 * kVerticalResolution))) + 1;
    vertical_voxel_size_ = 2 * half_bins + 1;
    vertical_angle_offset_ = -(90 - half_bins * kVerticalResolution);
    covered_voxel_.assign(kHorizontalVoxelSize * vertical_voxel_size_, 0.0f);
    reset_.assign(covered_voxel_.size(), true);
  }

  // Measured coverage requires an actual current-frame ray in the full-pose FOV.
  void ConfigureForwardSensor(double horizontal_deg, double vertical_deg, double range)
  {
    if (!std::isfinite(horizontal_deg) || horizontal_deg <= 0 || horizontal_deg > 360 ||
        !std::isfinite(vertical_deg) || vertical_deg <= 0 || vertical_deg >= 180 ||
        !std::isfinite(range) || range <= 0)
      throw std::invalid_argument("Invalid forward sensor FOV/range");
    directional_ = true;
    prediction_ = false;
    horizontal_half_fov_ = horizontal_deg * M_PI / 360;
    vertical_half_fov_ = vertical_deg * M_PI / 360;
    sensor_range_ = range;
    vertical_voxel_size_ = 91;
    vertical_angle_offset_ = 0;
    covered_voxel_.assign(kHorizontalVoxelSize * vertical_voxel_size_, 0.0f);
    reset_.assign(covered_voxel_.size(), true);
  }
  bool IsDirectional() const { return directional_; }

  template <class PointType>
  bool InSensorFOV(const PointType& point) const
  {
    Eigen::Vector3d delta(point.x - pose_.position.x, point.y - pose_.position.y,
                          point.z - pose_.position.z);
    const double range = delta.norm();
    Eigen::Quaterniond q(pose_.orientation.w, pose_.orientation.x,
                         pose_.orientation.y, pose_.orientation.z);
    if (!delta.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-8 ||
        range < kEpsilon || range > sensor_range_) return false;
    const Eigen::Vector3d local = q.normalized().conjugate() * delta;
    return std::abs(std::atan2(local.y(), local.x())) <= horizontal_half_fov_ + 1e-12 &&
           std::abs(std::atan2(local.z(), std::hypot(local.x(), local.y()))) <= vertical_half_fov_ + 1e-12;
  }
  /**
   * @brief Get the Visualization Cloud object
   * TODO
   * @param visualization_cloud
   * @param resol
   * @param max_range
   */
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& visualization_cloud, double resol = 0.2,
                             double max_range = 25.0) const;

  static void setCloudDWZResol(double cloud_dwz_resol)
  {
    pointcloud_resolution_ = cloud_dwz_resol * kCloudInflateRatio;
  }

  void setPose(const geometry_msgs::msg::Pose& pose)
  {
    pose_ = pose;
  }
  geometry_msgs::msg::Pose getPose()
  {
    return pose_;
  }
  void setPosition(const geometry_msgs::msg::Point& position)
  {
    pose_.position = position;
  }
  geometry_msgs::msg::Point getPosition() const
  {
    return pose_.position;
  }
  void SetHeight(double height)
  {
    pose_.position.z = height;
  }

private:
  /**
   * @brief convert subscripts to linear indices
   *
   * @param row_index row index
   * @param column_index column index
   * @return int linear index
   */
  int sub2ind(int row_index, int column_index) const;
  /**
   * @brief convert linear indices to subscripts
   *
   * @param ind linear index
   * @param row_index row index
   * @param column_index column index
   */
  void ind2sub(int ind, int& row_index, int& column_index) const;
  /**
   * @brief whether a number is close to zero
   *
   * @param x input number
   * @return true
   * @return false
   */
  bool isZero(double x) const
  {
    return std::abs(x) < kEpsilon;
  }

  /**
   * @brief Get the Horizontal Angle object
   * TODO
   * @param dx
   * @param dy
   * @return int
   */
  inline int GetHorizontalAngle(double dx, double dy) const
  {
    double horizontal_angle = (misc_utils_ns::ApproxAtan2(dy, dx) * kToDegreeConst + 180) / kHorizontalResolution;
    return static_cast<int>(round(horizontal_angle));
  }
  /**
   * @brief Get the Vertical Angle object
   * TODO
   * @param dz
   * @param distance_to_point
   * @return int
   */
  inline int GetVerticalAngle(double dz, double distance_to_point) const
  {
    double vertical_angle =
        (acos(std::max(-1.0, std::min(1.0, dz / distance_to_point))) * kToDegreeConst + vertical_angle_offset_) / kVerticalResolution;
    return static_cast<int>(round(vertical_angle));
  }
  /**
   * @brief Get the Horizontal Neighbor Num object
   * TODO
   * @param distance_to_point
   * @return int
   */
  inline int GetHorizontalNeighborNum(double distance_to_point) const
  {
    return static_cast<int>(ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kHorizontalResolution)) /
           2;
  }
  /**
   * @brief Get the Vertical Neighbor Num object
   * TODO
   * @param distance_to_point
   * @return int
   */
  inline int GetVerticalNeighborNum(double distance_to_point) const
  {
    return static_cast<int>(ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kVerticalResolution)) /
           2;
  }
  inline bool RowIndexInRange(int row_index) const
  {
    return row_index >= 0 && row_index < vertical_voxel_size_;
  }
  inline bool ColumnIndexInRange(int column_index) const
  {
    return column_index >= 0 && column_index < kHorizontalVoxelSize;
  }

  // Constant converting radian to degree
  static const double kToDegreeConst;
  // Constant converting degree to radian
  static const double kToRadianConst;
  // Threshold for checking if a number is close to zero
  static const double kEpsilon;
  // Ratio for inflating the cloud
  static const double kCloudInflateRatio;
  // Horizontal field-of-view in degrees
  static const int kHorizontalFOV = 360;
  // Vertical field-of-view in degrees
  static const int kVerticalFOV = 24;
  // Horizontal resolution
  static const int kHorizontalResolution = 2;
  // Vertical resolution
  static const int kVerticalResolution = 2;
  // Horizontal dimension of the voxel grid
  static const int kHorizontalVoxelSize = kHorizontalFOV / kHorizontalResolution;
  // Vertical dimension of the voxel grid
  static const int kVerticalVoxelSize = kVerticalFOV / kVerticalResolution;
  // Vertical angle offset, eg, angle [75, 105] -> indices [0, 30]
  static const int kVerticalAngleOffset = -(90 - kVerticalFOV / 2);
  // The distance that a ray can reach from the direction determined by the horizontal angle and vertical angle
  std::vector<float> covered_voxel_ = std::vector<float>(kHorizontalVoxelSize * kVerticalVoxelSize, 0.0f);
  // Whether a voxel is reset
  std::vector<bool> reset_ = std::vector<bool>(kHorizontalVoxelSize * kVerticalVoxelSize, true);
  int vertical_voxel_size_ = kVerticalVoxelSize;
  int vertical_angle_offset_ = kVerticalAngleOffset;
  bool directional_ = false;
  bool prediction_ = false;
  double horizontal_half_fov_ = M_PI;
  double vertical_half_fov_ = M_PI / 15;
  double sensor_range_ = 0.0;
  // Pose of the lidar model
  geometry_msgs::msg::Pose pose_;
};
}  // namespace lidar_model_ns