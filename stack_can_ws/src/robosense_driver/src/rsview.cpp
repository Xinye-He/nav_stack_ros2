// rsview.cpp
#include <memory>
#include <thread>
#include <chrono>
#include <cstring>  // memcpy

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

// 使用 PCL 或自定义点云
typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;
using namespace std::chrono_literals;

// 将 rs_driver 的时间戳转换为 ROS2 纳秒
inline int64_t to_ros_time_ns(double ts_sec) {           // 若为秒
  return static_cast<int64_t>(ts_sec * 1e9);
}
inline int64_t to_ros_time_ns(float ts_sec) {            // 若为秒（float）
  return static_cast<int64_t>(ts_sec * 1e9f);
}
inline int64_t to_ros_time_ns(uint64_t ts_us) {          // 若为微秒（常见）
  return static_cast<int64_t>(ts_us) * 1000LL;
}
inline int64_t to_ros_time_ns(int64_t ts_us) {           // 微秒（有符号）
  return ts_us * 1000LL;
}
inline int64_t to_ros_time_ns(unsigned long long ts_us) {// 微秒（unsigned long long）
  return static_cast<int64_t>(ts_us) * 1000LL;
}
inline int64_t to_ros_time_ns(long long ts_us) {         // 微秒（long long）
  return ts_us * 1000LL;
}

// 同步队列（用于内存池/双缓冲）
SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

// 全局发布器与日志器（供回调使用）
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
rclcpp::Logger logger = rclcpp::get_logger("robosense_driver");

// 驱动回调：从池里取空消息
std::shared_ptr<PointCloudMsg> driverGetPointCloudCallback() {
  auto msg = free_cloud_queue.pop();
  return msg ? msg : std::make_shared<PointCloudMsg>();
}

// 驱动回调：返回已填充点云
void driverReturnPointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
  stuffed_cloud_queue.push(msg);
}

// 异常回调
void exceptionCallback(const Error& code) {
  RCLCPP_WARN(logger, "RoboSense Driver Error: %s", code.toString().c_str());
}

class RoboSenseNode : public rclcpp::Node {
public:
  RoboSenseNode() : Node("robosense_driver") {
    logger = this->get_logger();

    // 参数配置
    this->declare_parameter<std::string>("frame_id", "rslidar");
    this->declare_parameter<int>("msop_port", 6699);
    this->declare_parameter<int>("difop_port", 7788);
    this->declare_parameter<int>("pool_size", 50);
    this->declare_parameter<bool>("use_lidar_timestamp", true);
    this->declare_parameter<bool>("use_lidar_clock", true);  // 新增

    frame_id_ = this->get_parameter("frame_id").as_string();
    int msop_port = this->get_parameter("msop_port").as_int();
    int difop_port = this->get_parameter("difop_port").as_int();
    int pool_size = this->get_parameter("pool_size").as_int();
    use_lidar_timestamp_ = this->get_parameter("use_lidar_timestamp").as_bool();
    bool use_lidar_clock_param = this->get_parameter("use_lidar_clock").as_bool();

    // 发布器（传感器 QoS）
    cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", rclcpp::SensorDataQoS());

    // 预分配点云对象池
    for (int i = 0; i < pool_size; ++i) {
      free_cloud_queue.push(std::make_shared<PointCloudMsg>());
    }

    // 初始化驱动参数
    RSDriverParam param;
    param.input_type = InputType::ONLINE_LIDAR;
    param.input_param.msop_port = msop_port;
    param.input_param.difop_port = difop_port;
    param.lidar_type = LidarType::RSE1;  // E1/E1R

    // 新增：优先使用雷达时钟，并确保等待 DIFOP
    param.decoder_param.use_lidar_clock = use_lidar_clock_param;
    param.decoder_param.wait_for_difop = true;

    param.print();

    // 创建并初始化驱动
    driver_.reset(new LidarDriver<PointCloudMsg>());
    driver_->regPointCloudCallback(driverGetPointCloudCallback, driverReturnPointCloudCallback);
    driver_->regExceptionCallback(exceptionCallback);

    if (!driver_->init(param)) {
      RCLCPP_FATAL(this->get_logger(), "Driver initialization failed!");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "RoboSense driver initialized successfully.");

    // 启动驱动
    driver_->start();
    RCLCPP_INFO(this->get_logger(), "RoboSense driver started.");

    // 启动点云处理线程
    processing_thread_ = std::thread(&RoboSenseNode::processClouds, this);
  }

  ~RoboSenseNode() override {
    // 先停驱动，避免继续往队列塞数据
    if (driver_) {
      driver_->stop();
    }
    // 推入一个空指针唤醒处理线程退出
    stuffed_cloud_queue.push(nullptr);

    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

private:
  // 将 RoboSense 点云转为 ROS2 PointCloud2
  sensor_msgs::msg::PointCloud2::SharedPtr convertToROS2PointCloud(
      const std::shared_ptr<PointCloudMsg>& rs_cloud) {
    auto cloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // 时间戳
    if (use_lidar_timestamp_) {
      // 兼容秒/微秒两种输入，统一转纳秒
      const int64_t ns = to_ros_time_ns(rs_cloud->timestamp);
      cloud2->header.stamp = rclcpp::Time(ns);
    } else {
      cloud2->header.stamp = this->now();
    }

    cloud2->header.frame_id = frame_id_;
    cloud2->height = 1;
    cloud2->width = static_cast<uint32_t>(rs_cloud->points.size());
    cloud2->is_dense = false;
    cloud2->is_bigendian = false;

    // 定义字段：x, y, z, intensity
    sensor_msgs::msg::PointField x_field, y_field, z_field, intensity_field;
    x_field.name = "x"; x_field.offset = 0; x_field.datatype = sensor_msgs::msg::PointField::FLOAT32; x_field.count = 1;
    y_field.name = "y"; y_field.offset = 4; y_field.datatype = sensor_msgs::msg::PointField::FLOAT32; y_field.count = 1;
    z_field.name = "z"; z_field.offset = 8; z_field.datatype = sensor_msgs::msg::PointField::FLOAT32; z_field.count = 1;
    intensity_field.name = "intensity"; intensity_field.offset = 12;
    intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32; intensity_field.count = 1;

    cloud2->fields.clear();
    cloud2->fields.push_back(x_field);
    cloud2->fields.push_back(y_field);
    cloud2->fields.push_back(z_field);
    cloud2->fields.push_back(intensity_field);

    cloud2->point_step = 16;
    cloud2->row_step = cloud2->point_step * cloud2->width;

    // 填充数据
    cloud2->data.resize(cloud2->row_step);
    for (size_t i = 0; i < rs_cloud->points.size(); ++i) {
      const auto& p = rs_cloud->points[i];
      std::memcpy(&cloud2->data[i * 16 + 0],  &p.x,         4);
      std::memcpy(&cloud2->data[i * 16 + 4],  &p.y,         4);
      std::memcpy(&cloud2->data[i * 16 + 8],  &p.z,         4);
      std::memcpy(&cloud2->data[i * 16 + 12], &p.intensity, 4);
    }

    return cloud2;
  }

  void processClouds() {
    double last_ts_s = 0.0;

    while (rclcpp::ok()) {
      auto rs_cloud = stuffed_cloud_queue.popWait();
      if (!rs_cloud) {
        // 收到析构发来的哨兵，退出线程
        break;
      }

      // 转换并发布
      auto ros_cloud = convertToROS2PointCloud(rs_cloud);
      cloud_pub->publish(*ros_cloud);

      // 调试：节流打印原始时间戳增量（自动识别单位，显示毫秒）
      if (use_lidar_timestamp_) {
        // 统一把任意单位(ts)估计转换为秒
        double v = static_cast<double>(rs_cloud->timestamp);
        double cur_s;
        if (v > 1e16) {            // 约纳秒
          cur_s = v * 1e-9;
        } else if (v > 1e12) {     // 约微秒
          cur_s = v * 1e-6;
        } else if (v > 1e8 && v < 1e10) { // 约秒（UNIX epoch）
          cur_s = v;
        } else {                   // 其他情况按秒处理
          cur_s = v;
        }
        double dt_ms = (last_ts_s > 0.0) ? (cur_s - last_ts_s) * 1e3 : 0.0;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "RS raw_ts≈%.6f s, dt=%.2f ms", cur_s, dt_ms);
        last_ts_s = cur_s;
      }

      RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", rs_cloud->points.size());

      // 归还对象到空闲池
      free_cloud_queue.push(rs_cloud);
    }
  }

  std::unique_ptr<LidarDriver<PointCloudMsg>> driver_;
  std::thread processing_thread_;

  std::string frame_id_;
  bool use_lidar_timestamp_{true};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoboSenseNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
