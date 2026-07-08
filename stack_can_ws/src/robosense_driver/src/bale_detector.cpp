#include <memory>
#include <cmath>
#include <limits>
#include <vector>
#include <array>
#include <unordered_set>
#include <sstream>
#include <cstdint>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "visualization_msgs/msg/marker_array.hpp"
#include "stack_msgs/msg/bale_target.hpp"

class BaleDetectorGround : public rclcpp::Node
{
public:
  BaleDetectorGround()
  : Node("bale_detector_ground")
  {
    // ===== 基本参数 =====
    min_range_ = this->declare_parameter<float>("min_range_m", 0.0f);
    max_range_ = this->declare_parameter<float>("max_range_m", 10.0f);

    // 固定外参：地面坐标系由安装参数直接给出
    sensor_height_m_ = this->declare_parameter<float>("sensor_height_m", 1.70f);
    mount_pitch_deg_ = this->declare_parameter<float>("mount_pitch_deg", -10.0f);
    mount_roll_deg_  = this->declare_parameter<float>("mount_roll_deg", 0.0f);

    // 高度带筛选：保留离地 [min_height, max_height] 的点
    min_height_m_ = this->declare_parameter<float>("min_height_m", 0.10f);
    max_height_m_ = this->declare_parameter<float>("max_height_m", 1.50f);

    // 2D聚类参数（在 ground 平面的 XY 上）
    cluster_tolerance_ = this->declare_parameter<float>("cluster_tolerance_m", 0.08f);
    cluster_min_size_  = this->declare_parameter<int>("cluster_min_size", 500);
    cluster_max_size_  = this->declare_parameter<int>("cluster_max_size", 1000);

    // 草捆二维尺寸判据（地平面 PCA 包围盒）
    bale_min_long_  = this->declare_parameter<float>("bale_min_long_m", 0.50f);
    bale_max_long_  = this->declare_parameter<float>("bale_max_long_m", 2.0f);
    bale_min_short_ = this->declare_parameter<float>("bale_min_short_m", 0.40f);
    bale_max_short_ = this->declare_parameter<float>("bale_max_short_m", 1.5f);

    // 长宽比
    aspect_min_ = this->declare_parameter<float>("aspect_min", 1.0f);
    aspect_max_ = this->declare_parameter<float>("aspect_max", 5.0f);

    // 候选在 z 方向的厚度
    z_span_min_ = this->declare_parameter<float>("z_span_min_m", 0.2f);
    z_span_max_ = this->declare_parameter<float>("z_span_max_m", 0.5f);

    // 一个简单的二维占据率，用于剔除特别稀疏/细碎的簇
    occupancy_grid_res_m_ = this->declare_parameter<float>("occupancy_grid_res_m", 0.08f);
    fill_ratio_min_       = this->declare_parameter<float>("fill_ratio_min", 0.18f);
    log_detection_results_ = this->declare_parameter<bool>("log_detection_results", true);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points",
      rclcpp::SensorDataQoS(),
      std::bind(&BaleDetectorGround::cloudCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "bale_markers", 10);

    ground_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground_aligned_points", 10);

    height_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "height_filtered_points", 10);

    projected_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "projected_cluster_points", 10);

    bale_target_pub_ = this->create_publisher<stack_msgs::msg::BaleTarget>(
      "bale_target", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "BaleDetectorGround started. fixed-ground mode enabled.");
  }

private:
  struct GroundFrame
  {
    Eigen::Vector3f x_g_in_lidar;
    Eigen::Vector3f y_g_in_lidar;
    Eigen::Vector3f z_g_in_lidar;
    Eigen::Vector3f origin_in_lidar;   // ground原点在lidar坐标中的位置
    Eigen::Matrix3f R_lg;              // 列向量是ground各轴在lidar中的表示
    Eigen::Matrix3f R_gl;              // lidar -> ground 旋转
  };

  struct Candidate2D
  {
    bool valid{false};

    Eigen::Vector2f center_xy = Eigen::Vector2f::Zero();
    float center_z = 0.0f;

    Eigen::Vector2f e_major = Eigen::Vector2f::UnitX();
    Eigen::Vector2f e_minor = Eigen::Vector2f::UnitY();

    float len_major = 0.0f;
    float len_minor = 0.0f;
    float z_min = 0.0f;
    float z_max = 0.0f;
    float z_span = 0.0f;
    float fill_ratio = 0.0f;

    std::array<Eigen::Vector2f, 4> corners;
  };

  static float deg2rad(float deg)
  {
    return deg * static_cast<float>(M_PI) / 180.0f;
  }

  GroundFrame buildGroundFrame() const
  {
    GroundFrame gf;

    // 约定：
    // ground: x前, y左, z上
    // lidar : 默认也按 x前, y左, z上理解
    //
    // 这里把“ground轴在lidar中的表示”构造出来。
    // 若发现 pitch 符号反了，只需把 launch/参数里的 mount_pitch_deg 改成相反数即可。
    const float roll  = deg2rad(mount_roll_deg_);
    const float pitch = deg2rad(mount_pitch_deg_);

    Eigen::AngleAxisf Rx(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());

    // 这里使用 Ry * Rx，把 ground 坐标轴旋到 lidar 坐标中
    Eigen::Matrix3f R_lg = (Ry * Rx).toRotationMatrix();

    gf.R_lg = R_lg;
    gf.R_gl = R_lg.transpose();

    gf.x_g_in_lidar = gf.R_lg.col(0);
    gf.y_g_in_lidar = gf.R_lg.col(1);
    gf.z_g_in_lidar = gf.R_lg.col(2);

    // ground原点设为“雷达在地面上的垂足”
    // ground z轴向上，则在 lidar 坐标里，地面原点位于雷达沿 -z_g 方向 sensor_height_m_ 处
    gf.origin_in_lidar = -sensor_height_m_ * gf.z_g_in_lidar;

    return gf;
  }

  Eigen::Vector3f pointLidarToGround(const Eigen::Vector3f & p_l, const GroundFrame & gf) const
  {
    return gf.R_gl * (p_l - gf.origin_in_lidar);
  }

  void publishGroundTF(const GroundFrame & gf, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "rslidar";
    tf_msg.child_frame_id = "ground";

    tf_msg.transform.translation.x = gf.origin_in_lidar.x();
    tf_msg.transform.translation.y = gf.origin_in_lidar.y();
    tf_msg.transform.translation.z = gf.origin_in_lidar.z();

    tf2::Matrix3x3 rot(
      gf.x_g_in_lidar.x(), gf.y_g_in_lidar.x(), gf.z_g_in_lidar.x(),
      gf.x_g_in_lidar.y(), gf.y_g_in_lidar.y(), gf.z_g_in_lidar.y(),
      gf.x_g_in_lidar.z(), gf.y_g_in_lidar.z(), gf.z_g_in_lidar.z());

    tf2::Quaternion q;
    rot.getRotation(q);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  static void computeMeanCov2D(
    const std::vector<Eigen::Vector2f> & pts,
    Eigen::Vector2f & mean,
    Eigen::Matrix2f & cov)
  {
    mean.setZero();
    cov.setZero();

    if (pts.empty()) {
      return;
    }

    for (const auto & p : pts) {
      mean += p;
    }
    mean /= static_cast<float>(pts.size());

    for (const auto & p : pts) {
      Eigen::Vector2f d = p - mean;
      cov += d * d.transpose();
    }
    cov /= std::max(1.0f, static_cast<float>(pts.size() - 1));
  }

  float estimateFillRatio(
    const std::vector<Eigen::Vector2f> & local_pts,
    float major_min, float major_max,
    float minor_min, float minor_max) const
  {
    if (local_pts.empty()) {
      return 0.0f;
    }

    const float box_w = std::max(major_max - major_min, 1e-3f);
    const float box_h = std::max(minor_max - minor_min, 1e-3f);
    const float box_area = box_w * box_h;

    if (box_area < 1e-6f) {
      return 0.0f;
    }

    const float res = std::max(occupancy_grid_res_m_, 0.02f);
    std::unordered_set<std::uint64_t> occupied;
    occupied.reserve(local_pts.size());

    for (const auto & p : local_pts) {
      int ix = static_cast<int>(std::floor((p.x() - major_min) / res));
      int iy = static_cast<int>(std::floor((p.y() - minor_min) / res));
      std::uint64_t key =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) ^
        static_cast<std::uint32_t>(iy);
      occupied.insert(key);
    }

    const float occ_area = static_cast<float>(occupied.size()) * res * res;
    return occ_area / box_area;
  }

  Candidate2D analyzeCluster2D(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_height_filtered,
    const pcl::PointIndices & cluster_idx) const
  {
    Candidate2D out;
    if (cluster_idx.indices.empty()) {
      return out;
    }

    std::vector<Eigen::Vector2f> pts2;
    pts2.reserve(cluster_idx.indices.size());

    float sum_z = 0.0f;
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();

    for (int idx : cluster_idx.indices) {
      const auto & p = cloud_height_filtered->points[idx];
      pts2.emplace_back(p.x, p.y);
      sum_z += p.z;
      z_min = std::min(z_min, p.z);
      z_max = std::max(z_max, p.z);
    }

    if (pts2.size() < 3) {
      return out;
    }

    Eigen::Vector2f mean;
    Eigen::Matrix2f cov;
    computeMeanCov2D(pts2, mean, cov);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
    if (es.info() != Eigen::Success) {
      return out;
    }

    Eigen::Vector2f e_minor = es.eigenvectors().col(0).normalized();
    Eigen::Vector2f e_major = es.eigenvectors().col(1).normalized();

    if (e_major.x() < 0.0f) {
      e_major = -e_major;
    }
    e_minor = Eigen::Vector2f(-e_major.y(), e_major.x());

    float major_min = std::numeric_limits<float>::max();
    float major_max = std::numeric_limits<float>::lowest();
    float minor_min = std::numeric_limits<float>::max();
    float minor_max = std::numeric_limits<float>::lowest();

    std::vector<Eigen::Vector2f> local_pts;
    local_pts.reserve(pts2.size());

    for (const auto & p : pts2) {
      Eigen::Vector2f d = p - mean;
      float u = d.dot(e_major);
      float v = d.dot(e_minor);

      local_pts.emplace_back(u, v);

      major_min = std::min(major_min, u);
      major_max = std::max(major_max, u);
      minor_min = std::min(minor_min, v);
      minor_max = std::max(minor_max, v);
    }

    const float len_major = major_max - major_min;
    const float len_minor = minor_max - minor_min;
    const float aspect = len_major / std::max(len_minor, 1e-3f);
    const float z_span = z_max - z_min;
    const float fill_ratio = estimateFillRatio(local_pts, major_min, major_max, minor_min, minor_max);

    if (len_major < bale_min_long_ || len_major > bale_max_long_) {
      RCLCPP_INFO(
        this->get_logger(),
        "reject cluster: len_major=%.2f not in [%.2f, %.2f], len_minor=%.2f, aspect=%.2f, z_span=%.2f, fill=%.2f",
        len_major, bale_min_long_, bale_max_long_,
        len_minor, aspect, z_span, fill_ratio);
      return out;
    }

    if (len_minor < bale_min_short_ || len_minor > bale_max_short_) {
      RCLCPP_INFO(
        this->get_logger(),
        "reject cluster: len_minor=%.2f not in [%.2f, %.2f], len_major=%.2f, aspect=%.2f, z_span=%.2f, fill=%.2f",
        len_minor, bale_min_short_, bale_max_short_,
        len_major, aspect, z_span, fill_ratio);
      return out;
    }

    if (aspect < aspect_min_ || aspect > aspect_max_) {
      RCLCPP_INFO(
        this->get_logger(),
        "reject cluster: aspect=%.2f not in [%.2f, %.2f], size=(%.2f, %.2f), z_span=%.2f, fill=%.2f",
        aspect, aspect_min_, aspect_max_,
        len_major, len_minor, z_span, fill_ratio);
      return out;
    }

    if (z_span < z_span_min_ || z_span > z_span_max_) {
      RCLCPP_INFO(
        this->get_logger(),
        "reject cluster: z_span=%.2f not in [%.2f, %.2f], size=(%.2f, %.2f), aspect=%.2f, fill=%.2f",
        z_span, z_span_min_, z_span_max_,
        len_major, len_minor, aspect, fill_ratio);
      return out;
    }

    if (fill_ratio < fill_ratio_min_) {
      RCLCPP_INFO(
        this->get_logger(),
        "reject cluster: fill_ratio=%.2f < %.2f, size=(%.2f, %.2f), aspect=%.2f, z_span=%.2f",
        fill_ratio, fill_ratio_min_,
        len_major, len_minor, aspect, z_span);
      return out;
    }

    Eigen::Vector2f c1 = mean + e_major * major_min + e_minor * minor_min;
    Eigen::Vector2f c2 = mean + e_major * major_max + e_minor * minor_min;
    Eigen::Vector2f c3 = mean + e_major * major_max + e_minor * minor_max;
    Eigen::Vector2f c4 = mean + e_major * major_min + e_minor * minor_max;

    out.valid = true;
    out.center_xy = mean;
    out.center_z = sum_z / static_cast<float>(pts2.size());
    out.e_major = e_major;
    out.e_minor = e_minor;
    out.len_major = len_major;
    out.len_minor = len_minor;
    out.z_min = z_min;
    out.z_max = z_max;
    out.z_span = z_span;
    out.fill_ratio = fill_ratio;
    out.corners = {c1, c2, c3, c4};

    return out;
  }

  void add2DBoxMarker(
    visualization_msgs::msg::MarkerArray & arr,
    const Candidate2D & c,
    int & marker_id,
    const rclcpp::Time & stamp,
    bool is_best) const
  {
    visualization_msgs::msg::Marker box;
    box.header.stamp = stamp;
    box.header.frame_id = "ground";
    box.ns = "bale_2d_boxes";
    box.id = marker_id++;
    box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.orientation.w = 1.0;
    box.scale.x = is_best ? 0.06 : 0.03;
    box.color.r = is_best ? 1.0f : 0.1f;
    box.color.g = is_best ? 0.2f : 1.0f;
    box.color.b = is_best ? 0.1f : 0.1f;
    box.color.a = 0.95f;
    box.lifetime = rclcpp::Duration(0, 300000000);

    const float z_draw = 0.05f;

    auto push_pt = [&](const Eigen::Vector2f & p) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = z_draw;
      box.points.push_back(gp);
    };

    push_pt(c.corners[0]);
    push_pt(c.corners[1]);
    push_pt(c.corners[2]);
    push_pt(c.corners[3]);
    push_pt(c.corners[0]);

    arr.markers.push_back(box);

    visualization_msgs::msg::Marker center;
    center.header = box.header;
    center.ns = "bale_2d_centers";
    center.id = marker_id++;
    center.type = visualization_msgs::msg::Marker::SPHERE;
    center.action = visualization_msgs::msg::Marker::ADD;
    center.pose.position.x = c.center_xy.x();
    center.pose.position.y = c.center_xy.y();
    center.pose.position.z = z_draw + 0.03f;
    center.pose.orientation.w = 1.0;
    center.scale.x = 0.10;
    center.scale.y = 0.10;
    center.scale.z = 0.10;
    center.color.r = is_best ? 1.0f : 0.9f;
    center.color.g = is_best ? 0.2f : 0.9f;
    center.color.b = 0.0f;
    center.color.a = 0.9f;
    center.lifetime = rclcpp::Duration(0, 300000000);
    arr.markers.push_back(center);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_in);

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, nan_indices);
    cloud_in->is_dense = true;

    if (cloud_in->empty()) {
      publishEmptyAll(msg->header.stamp);
      return;
    }

    const GroundFrame gf = buildGroundFrame();
    publishGroundTF(gf, msg->header.stamp);

    // 1) 先做前向 ROI，然后整体转到 ground
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ground_cloud->reserve(cloud_in->size());

    for (const auto & p : cloud_in->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      // 先用 lidar 原坐标的前方范围做一次粗筛，减少计算量
      if (p.x <= 0.0f) {
        continue;
      }

      const float r_xy_lidar = std::sqrt(p.x * p.x + p.y * p.y);
      if (r_xy_lidar < min_range_ || r_xy_lidar > max_range_) {
        continue;
      }

      Eigen::Vector3f p_l(p.x, p.y, p.z);
      Eigen::Vector3f p_g = pointLidarToGround(p_l, gf);

      pcl::PointXYZI q;
      q.x = p_g.x();
      q.y = p_g.y();
      q.z = p_g.z();
      q.intensity = p.intensity;
      ground_cloud->points.push_back(q);
    }

    ground_cloud->width = static_cast<std::uint32_t>(ground_cloud->points.size());
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    publishPointCloud<pcl::PointXYZI>(
      ground_cloud_pub_, ground_cloud, rclcpp::Time(msg->header.stamp), "ground");

    if (ground_cloud->points.size() < static_cast<size_t>(cluster_min_size_)) {
      publishEmptyMarkers(msg->header.stamp);
      publishEmptyProjectedClusterCloud(msg->header.stamp);
      publishEmptyHeightFilteredCloud(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    // 2) 按离地高度筛选
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    height_filtered->reserve(ground_cloud->size());

    for (const auto & p : ground_cloud->points) {
      const float r_xy_ground = std::sqrt(p.x * p.x + p.y * p.y);
      if (r_xy_ground < min_range_ || r_xy_ground > max_range_) {
        continue;
      }

      if (p.z >= min_height_m_ && p.z <= max_height_m_) {
        height_filtered->points.push_back(p);
      }
    }

    height_filtered->width = static_cast<std::uint32_t>(height_filtered->points.size());
    height_filtered->height = 1;
    height_filtered->is_dense = true;

    publishPointCloud<pcl::PointXYZI>(
      height_filtered_pub_, height_filtered, rclcpp::Time(msg->header.stamp), "ground");

    if (height_filtered->points.size() < static_cast<size_t>(cluster_min_size_)) {
      publishEmptyMarkers(msg->header.stamp);
      publishEmptyProjectedClusterCloud(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    // 3) 2D聚类：直接把点投影到地面，只在 ground XY 上聚类
    pcl::PointCloud<pcl::PointXYZI>::Ptr projected_for_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    projected_for_cluster->reserve(height_filtered->size());

    for (const auto & p : height_filtered->points) {
      pcl::PointXYZI q;
      q.x = p.x;
      q.y = p.y;
      q.z = 0.0f;
      q.intensity = p.intensity;
      projected_for_cluster->points.push_back(q);
    }

    projected_for_cluster->width = static_cast<std::uint32_t>(projected_for_cluster->points.size());
    projected_for_cluster->height = 1;
    projected_for_cluster->is_dense = true;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(projected_for_cluster);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(projected_for_cluster);
    ec.extract(cluster_indices);

    // 彩色投影点云，方便看每个候选簇
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projected_cluster_cloud->reserve(projected_for_cluster->size());

    visualization_msgs::msg::MarkerArray marker_array;
    {
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.stamp = msg->header.stamp;
      clear_marker.header.frame_id = "ground";
      clear_marker.ns = "bale_2d_boxes";
      clear_marker.id = 0;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);
    }

    if (cluster_indices.empty()) {
      marker_pub_->publish(marker_array);
      publishEmptyProjectedClusterCloud(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    bool found_any = false;
    int marker_id = 1;
    int cluster_id = 0;

    float best_R = std::numeric_limits<float>::max();
    float best_angle_user = 0.0f;
    Candidate2D best_candidate;

    std::vector<Candidate2D> valid_candidates;
    valid_candidates.reserve(cluster_indices.size());

    for (const auto & cluster : cluster_indices) {
      if (cluster.indices.empty()) {
        ++cluster_id;
        continue;
      }

      const uint8_t r = static_cast<uint8_t>((cluster_id * 53) % 256);
      const uint8_t g = static_cast<uint8_t>((cluster_id * 97) % 256);
      const uint8_t b = static_cast<uint8_t>((cluster_id * 151) % 256);

      for (int idx : cluster.indices) {
        const auto & p = projected_for_cluster->points[idx];
        pcl::PointXYZRGB pc;
        pc.x = p.x;
        pc.y = p.y;
        pc.z = 0.02f;
        pc.r = r;
        pc.g = g;
        pc.b = b;
        projected_cluster_cloud->points.push_back(pc);
      }

      Candidate2D c = analyzeCluster2D(height_filtered, cluster);
      ++cluster_id;

      if (!c.valid) {
        continue;
      }

      const float R = std::sqrt(c.center_xy.x() * c.center_xy.x() +
                                c.center_xy.y() * c.center_xy.y());
      if (R < min_range_ || R > max_range_) {
        continue;
      }

      // ROS通常左正右负；你现在沿用原项目“左负右正”
      const float yaw_ros = std::atan2(c.center_xy.y(), c.center_xy.x());
      const float angle_user = -yaw_ros;

      if (R < best_R) {
        best_R = R;
        best_angle_user = angle_user;
        best_candidate = c;
      }

      valid_candidates.push_back(c);
      found_any = true;
    }

    projected_cluster_cloud->width = static_cast<std::uint32_t>(projected_cluster_cloud->points.size());
    projected_cluster_cloud->height = 1;
    projected_cluster_cloud->is_dense = true;

    publishPointCloud<pcl::PointXYZRGB>(
      projected_cluster_pub_, projected_cluster_cloud, rclcpp::Time(msg->header.stamp), "ground");

    if (!found_any) {
      marker_pub_->publish(marker_array);
      publishEmptyTarget();
      return;
    }

    // 先画所有候选，再高亮最近那个
    for (const auto & c : valid_candidates) {
      const bool is_best =
        (std::abs(c.center_xy.x() - best_candidate.center_xy.x()) < 1e-4f) &&
        (std::abs(c.center_xy.y() - best_candidate.center_xy.y()) < 1e-4f);
      add2DBoxMarker(marker_array, c, marker_id, msg->header.stamp, is_best);
    }

    marker_pub_->publish(marker_array);

    stack_msgs::msg::BaleTarget tgt;
    tgt.distance_m = best_R;
    tgt.angle_deg  = best_angle_user * 180.0f / static_cast<float>(M_PI);
    tgt.valid      = true;
    bale_target_pub_->publish(tgt);

    if (log_detection_results_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Nearest bale: R=%.2f m, angle=%.1f deg, center=(%.2f, %.2f), "
        "size=(%.2f, %.2f), z_span=%.2f, fill=%.2f",
        best_R,
        tgt.angle_deg,
        best_candidate.center_xy.x(),
        best_candidate.center_xy.y(),
        best_candidate.len_major,
        best_candidate.len_minor,
        best_candidate.z_span,
        best_candidate.fill_ratio);
    }
  }

  template<typename PointT>
  void publishPointCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const typename pcl::PointCloud<PointT>::Ptr & cloud,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    if (!pub) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*cloud, out);
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    pub->publish(out);
  }

  void publishEmptyCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    if (!pub) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = 0;
    msg.is_dense = true;
    pub->publish(msg);
  }

  void publishEmptyMarkers(const rclcpp::Time & stamp)
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = stamp;
    clear_marker.header.frame_id = "ground";
    clear_marker.ns = "bale_2d_boxes";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear_marker);

    marker_pub_->publish(arr);
  }

  void publishEmptyHeightFilteredCloud(const rclcpp::Time & stamp)
  {
    publishEmptyCloud(height_filtered_pub_, stamp, "ground");
  }

  void publishEmptyProjectedClusterCloud(const rclcpp::Time & stamp)
  {
    publishEmptyCloud(projected_cluster_pub_, stamp, "ground");
  }

  void publishEmptyTarget()
  {
    if (!bale_target_pub_) {
      return;
    }
    stack_msgs::msg::BaleTarget tgt;
    tgt.distance_m = 0.0f;
    tgt.angle_deg = 0.0f;
    tgt.valid = false;
    bale_target_pub_->publish(tgt);
  }

  void publishEmptyAll(const rclcpp::Time & stamp)
  {
    publishEmptyMarkers(stamp);
    publishEmptyCloud(ground_cloud_pub_, stamp, "ground");
    publishEmptyCloud(height_filtered_pub_, stamp, "ground");
    publishEmptyCloud(projected_cluster_pub_, stamp, "ground");
    publishEmptyTarget();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr height_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_cluster_pub_;
  rclcpp::Publisher<stack_msgs::msg::BaleTarget>::SharedPtr bale_target_pub_;

  float min_range_;
  float max_range_;

  float sensor_height_m_;
  float mount_pitch_deg_;
  float mount_roll_deg_;

  float min_height_m_;
  float max_height_m_;

  float cluster_tolerance_;
  int cluster_min_size_;
  int cluster_max_size_;

  float bale_min_long_;
  float bale_max_long_;
  float bale_min_short_;
  float bale_max_short_;

  float aspect_min_;
  float aspect_max_;

  float z_span_min_;
  float z_span_max_;

  float occupancy_grid_res_m_;
  float fill_ratio_min_;
  bool log_detection_results_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaleDetectorGround>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
