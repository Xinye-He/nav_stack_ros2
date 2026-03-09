#include <memory>
#include <cmath>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>                 // removeNaNFromPointCloud
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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
    // 基本参数（可改成 declare_parameter 再从参数服务器读取）
    min_range_ = 1.0f;   // 关注的水平距离范围
    max_range_ = 10.0f;

    // 草捆尺寸（直径0.6m、长1.2m，对包围盒给宽松范围）
    bale_min_long_ = 0.3f;   // XY平面包围盒较长边下限
    bale_max_long_ = 2.0f;   // 较长边上限
    bale_min_short_ = 0.1f;  // 较短边下限
    bale_max_short_ = 2.0f;  // 较短边上限

    cluster_tolerance_ = 0.3f;   // 聚类半径
    cluster_min_size_ = 100;
    cluster_max_size_ = 5000;

    ground_dist_thresh_ = 0.21f; // 地面RANSAC拟合距离阈值（m）

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points",
      rclcpp::SensorDataQoS(),
      std::bind(&BaleDetectorGround::cloudCallback, this, std::placeholders::_1));

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "bale_markers", 10);

    // 新增：去地面点云
    nonground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "nonground_points", 10);

    // 新增：聚类后着色点云
    cluster_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cluster_points", 10);

    // 新增：最近草捆目标
    bale_target_pub_ = this->create_publisher<stack_msgs::msg::BaleTarget>(
      "bale_target", 10);

    RCLCPP_INFO(this->get_logger(),
                "BaleDetectorGround subscribed to /rslidar_points");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      return;
    }

    // 1. ROS2 -> PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // 1.1 去掉 NaN 点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    cloud_in->is_dense = true;

    if (cloud_in->empty()) {
      publishEmptyMarkers(msg->header.stamp);
      return;
    }

    // 2. 前方 设定距离 的 ROI（在雷达坐标系 XY 平面上）
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_roi->reserve(cloud_in->size());

    for (const auto & p : cloud_in->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      // 假设 x>0 为前方
      if (p.x <= 0.0f) continue;

      float r_xy = std::sqrt(p.x * p.x + p.y * p.y);
      if (r_xy < min_range_ || r_xy > max_range_) continue;

      cloud_roi->points.push_back(p);
    }
    cloud_roi->width = static_cast<uint32_t>(cloud_roi->points.size());
    cloud_roi->height = 1;
    cloud_roi->is_dense = true;

    if (cloud_roi->points.size() < static_cast<size_t>(cluster_min_size_)) {
      publishEmptyMarkers(msg->header.stamp);
      // 同时也清空聚类点云
      publishEmptyPointClouds(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    // 3. 用 RANSAC 在 ROI 里拟合“地面平面”
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_dist_thresh_);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud_roi);

    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.segment(*ground_inliers, *coefficients);

    if (ground_inliers->indices.empty() || coefficients->values.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No ground plane found, skip this frame.");
      publishEmptyMarkers(msg->header.stamp);
      publishEmptyPointClouds(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    // 平面方程: a x + b y + c z + d = 0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    Eigen::Vector3f n_raw(a, b, c);
    float n_norm = n_raw.norm();
    if (n_norm < 1e-6f) {
      publishEmptyMarkers(msg->header.stamp);
      publishEmptyPointClouds(msg->header.stamp);
      return;
    }

    Eigen::Vector3f n = n_raw / n_norm;  // 单位法向
    float d_norm = d / n_norm;

    // 用地面内点的质心，调整法向方向：保证法向从地面指向雷达
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
    for (int idx : ground_inliers->indices) {
      const auto & p = cloud_roi->points[idx];
      centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= static_cast<float>(ground_inliers->indices.size());

    Eigen::Vector3f v_plane_to_sensor = -centroid;  // 质心->传感器(0,0,0)
    if (n.dot(v_plane_to_sensor) < 0.0f) {
      // 法向朝“地下”，翻转
      n = -n;
      d_norm = -d_norm;
    }

    // 这里 n 就是“向上”的地面法向，d_norm 为平面到原点的有符号距离

    // 3.1 构造“水平地面坐标系 ground”在 rslidar 坐标系下的基
    Eigen::Vector3f z_g = n;  // 向上

    // x_g: 雷达x轴在地面平面的投影（前方）
    Eigen::Vector3f ex(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f x_g = ex - ex.dot(z_g) * z_g;
    if (x_g.norm() < 1e-3f) {
      // 退化情况（极不可能），用 y 轴投影
      ex = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
      x_g = ex - ex.dot(z_g) * z_g;
    }
    x_g.normalize();

    // y_g: 右手系，前+x，左+y，上+z
    Eigen::Vector3f y_g = z_g.cross(x_g);
    y_g.normalize();

    // 3.2 地平面上选择一个原点：传感器在地面上的垂足
    // 实际地面平面: n·p + d_norm = 0
    // 原点(0,0,0)在rslidar，垂足 p0 = -d_norm * n
    Eigen::Vector3f p0 = -d_norm * n;

    // 3.3 发布 TF: rslidar -> ground
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "rslidar";   // 父坐标系
    tf_msg.child_frame_id  = "ground";    // 子坐标系

    tf_msg.transform.translation.x = p0.x();
    tf_msg.transform.translation.y = p0.y();
    tf_msg.transform.translation.z = p0.z();

    // 旋转矩阵（列为子坐标轴在父坐标系中的坐标）
    tf2::Matrix3x3 rot(
      x_g.x(), y_g.x(), z_g.x(),
      x_g.y(), y_g.y(), z_g.y(),
      x_g.z(), y_g.z(), z_g.z()
    );
    tf2::Quaternion q;
    rot.getRotation(q);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);

    // 4. 去掉地面点，得到非地面点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZI>);
    {
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(cloud_roi);
      extract.setIndices(ground_inliers);
      extract.setNegative(true);  // 提取非地面
      extract.filter(*cloud_nonground);
    }

    if (cloud_nonground->empty()) {
      publishEmptyMarkers(msg->header.stamp);
      publishEmptyPointClouds(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    // 4.1 发布去地面点云（供RViz中单独查看）
    {
      sensor_msgs::msg::PointCloud2 nonground_msg;
      pcl::toROSMsg(*cloud_nonground, nonground_msg);
      nonground_msg.header = msg->header;       // 使用原来的时间戳和 frame_id=rslidar
      nonground_pub_->publish(nonground_msg);
    }

    if (cloud_nonground->points.size() < static_cast<size_t>(cluster_min_size_)) {
      publishEmptyMarkers(msg->header.stamp);
      // 聚类点云暂时发空
      publishEmptyClusterCloud(msg->header.stamp);
      return;
    }

    // 5. 欧式聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_nonground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_nonground);
    ec.extract(cluster_indices);

    // 为可视化聚类效果，构造一个带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cluster_cloud->reserve(cloud_nonground->points.size());

    // MarkerArray：本帧所有草捆候选的可视化
    visualization_msgs::msg::MarkerArray marker_array;

    // 第一个 Marker: DELETEALL，清空旧的
    {
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.stamp = msg->header.stamp;
      clear_marker.header.frame_id = "rslidar";
      clear_marker.ns = "bale_candidates";
      clear_marker.id = 0;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);
    }

    if (cluster_indices.empty()) {
      marker_pub_->publish(marker_array);
      publishEmptyClusterCloud(msg->header.stamp);
      publishEmptyTarget();
      return;
    }

    // 6. 遍历所有cluster，筛选“草捆候选”，再用物理水平距离/角度选最近的
    bool found_candidate = false;
    float best_R = std::numeric_limits<float>::max();
    float best_angle_user = 0.0f;
    Eigen::Vector3f best_centroid_L(0.0f, 0.0f, 0.0f);

    int marker_id = 1;
    int cluster_id = 0;

    for (const auto & indices_cluster : cluster_indices) {
      if (indices_cluster.indices.empty()) {
        cluster_id++;
        continue;
      }

      // 为该cluster生成一个颜色（简单hash）
      uint8_t r = static_cast<uint8_t>((cluster_id * 53) % 256);
      uint8_t g = static_cast<uint8_t>((cluster_id * 97) % 256);
      uint8_t b = static_cast<uint8_t>((cluster_id * 151) % 256);
      cluster_id++;

      // 6.1 计算 聚类质心 + 包围盒（在 rslidar 坐标系）
      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();

      // 同时构造带颜色的聚类点云
      for (int idx : indices_cluster.indices) {
        const auto & p = cloud_nonground->points[idx];

        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;

        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
        if (p.z < min_z) min_z = p.z;
        if (p.z > max_z) max_z = p.z;

        pcl::PointXYZRGB pc;
        pc.x = p.x;
        pc.y = p.y;
        pc.z = p.z;
        pc.r = r;
        pc.g = g;
        pc.b = b;
        cluster_cloud->points.push_back(pc);
      }

      const size_t N = indices_cluster.indices.size();
      float cx = sum_x / static_cast<float>(N);
      float cy = sum_y / static_cast<float>(N);
      float cz = sum_z / static_cast<float>(N);
      Eigen::Vector3f c_L(cx, cy, cz);

      float dx = max_x - min_x;
      float dy = max_y - min_y;
      float dz = max_z - min_z;

      // 6.2 XY平面包围盒长短边（只用于粗略尺寸过滤）
      float d_long = std::max(dx, dy);
      float d_short = std::min(dx, dy);

      // 草捆尺寸初筛
      if (d_long < bale_min_long_ || d_long > bale_max_long_) continue;
      if (d_short < bale_min_short_ || d_short > bale_max_short_) continue;
      if (dz < 0.53f || dz > 0.8f) continue;  // 高度粗过滤

      // 6.3 在“物理水平地面”坐标系下计算水平距离 + 角度
      // 去掉沿地面法向的分量：投影到通过传感器的、与地面平行的平面上
      float h = c_L.dot(z_g);                 // 高度分量
      Eigen::Vector3f v_h = c_L - h * z_g;    // 水平向量（在rslidar坐标中）

      // 在 ground 坐标中的分量
      float x_g_coord = v_h.dot(x_g);   // 前方 >0
      float y_g_coord = v_h.dot(y_g);   // 左 >0，右 <0（ROS惯例）

      float R = std::sqrt(x_g_coord * x_g_coord + y_g_coord * y_g_coord);

      // 物理水平距离范围过滤一次
      if (R < min_range_ || R > max_range_) continue;

      // ROS 惯例下的方位角：左正右负
      float yaw_ros = std::atan2(y_g_coord, x_g_coord);
      // 你的约定：左负右正 → 取负号
      float angle_user = -yaw_ros;

      // 更新最近目标
      if (R < best_R) {
        best_R = R;
        best_angle_user = angle_user;
        best_centroid_L = c_L;
        found_candidate = true;
      }

      // === 生成该草捆候选的可视化 Marker（CUBE 包围盒） ===
      visualization_msgs::msg::Marker m;
      m.header.stamp = msg->header.stamp;
      m.header.frame_id = "rslidar";     // RViz 会用 TF 转到 ground
      m.ns = "bale_candidates";
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // 包围盒中心用质心
      m.pose.position.x = cx;
      m.pose.position.y = cy;
      m.pose.position.z = cz;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.orientation.w = 1.0;

      // 包围盒尺寸
      m.scale.x = dx;
      m.scale.y = dy;
      m.scale.z = dz;

      // 颜色：绿色半透明表示草捆候选
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.color.a = 0.5f;

      m.lifetime = rclcpp::Duration(0, 500000000); // 0.5s

      marker_array.markers.push_back(m);

      // （可选）再加一个小球表示质心
      visualization_msgs::msg::Marker center;
      center.header = m.header;
      center.ns = "bale_centers";
      center.id = marker_id++;
      center.type = visualization_msgs::msg::Marker::SPHERE;
      center.action = visualization_msgs::msg::Marker::ADD;
      center.pose.position.x = cx;
      center.pose.position.y = cy;
      center.pose.position.z = cz;
      center.pose.orientation.w = 1.0;
      center.scale.x = 0.1;
      center.scale.y = 0.1;
      center.scale.z = 0.1;
      center.color.r = 1.0f;
      center.color.g = 0.0f;
      center.color.b = 0.0f;
      center.color.a = 0.8f;
      center.lifetime = rclcpp::Duration(0, 500000000);

      marker_array.markers.push_back(center);
    }

    // 完成聚类着色点云
    cluster_cloud->width = static_cast<uint32_t>(cluster_cloud->points.size());
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = true;

    // 发布可视化 Marker
    marker_pub_->publish(marker_array);

    // 发布聚类着色点云
    {
      sensor_msgs::msg::PointCloud2 cluster_msg;
      pcl::toROSMsg(*cluster_cloud, cluster_msg);
      cluster_msg.header = msg->header;   // frame_id=rslidar
      cluster_cloud_pub_->publish(cluster_msg);
    }

    // 7. 打印最近草捆候选的距离和角度
    if (found_candidate) {
      float angle_deg = best_angle_user * 180.0f / static_cast<float>(M_PI);

      // 发布到 /bale_target
      if (bale_target_pub_) {
        stack_msgs::msg::BaleTarget tgt;
        tgt.distance_m = best_R;
        tgt.angle_deg  = angle_deg;   // 注意：左负右正，与你栈内 angle_deg 约定一致
        tgt.valid      = true;
        bale_target_pub_->publish(tgt);
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Nearest bale (ground frame): R=%.2f m, angle_user=%.1f deg (left-, right+), "
        "centroid_L=(%.2f, %.2f, %.2f)",
        best_R, angle_deg,
        best_centroid_L.x(), best_centroid_L.y(), best_centroid_L.z());
    } else {
      // 虽然上面各处 early-return 已经发过 invalid，这里兜底一次也可以
      publishEmptyTarget();
    }
  }

  // 发布一个只带 DELETEALL 的 MarkerArray，用于清空 RViz 中旧的草捆显示
  void publishEmptyMarkers(const rclcpp::Time & stamp)
  {
    if (!marker_pub_) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = stamp;
    clear_marker.header.frame_id = "rslidar";
    clear_marker.ns = "bale_candidates";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear_marker);
    marker_pub_->publish(arr);
  }

  // 清空点云可视化（非必须，这里只是发布一个空消息）
  void publishEmptyPointClouds(const rclcpp::Time & stamp)
  {
    publishEmptyCloudOnPub(nonground_pub_, stamp);
    publishEmptyCloudOnPub(cluster_cloud_pub_, stamp);
  }

  void publishEmptyClusterCloud(const rclcpp::Time & stamp)
  {
    publishEmptyCloudOnPub(cluster_cloud_pub_, stamp);
  }

  void publishEmptyCloudOnPub(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
      const rclcpp::Time & stamp)
  {
    if (!pub) return;
    sensor_msgs::msg::PointCloud2 empty_msg;
    empty_msg.header.stamp = stamp;
    empty_msg.header.frame_id = "rslidar";
    empty_msg.height = 1;
    empty_msg.width = 0;
    empty_msg.is_dense = true;
    pub->publish(empty_msg);
  }

  void publishEmptyTarget()
  {
    if (!bale_target_pub_) return;
    stack_msgs::msg::BaleTarget msg;
    msg.distance_m = 0.0f;
    msg.angle_deg  = 0.0f;
    msg.valid      = false;
    bale_target_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 去地面点云 & 聚类点云发布 & 最近草捆目标
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_pub_;
  rclcpp::Publisher<stack_msgs::msg::BaleTarget>::SharedPtr bale_target_pub_;

  float min_range_;
  float max_range_;

  float bale_min_long_;
  float bale_max_long_;
  float bale_min_short_;
  float bale_max_short_;

  float cluster_tolerance_;
  int cluster_min_size_;
  int cluster_max_size_;

  float ground_dist_thresh_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaleDetectorGround>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
