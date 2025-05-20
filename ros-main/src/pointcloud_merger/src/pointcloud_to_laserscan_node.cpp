#include <memory>
#include <vector>
#include <cmath>
#include <limits>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
    PointCloudToLaserScanNode()
    : Node("pointcloud_to_laserscan_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // QoS 设置为 Reliable
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();

        // 订阅 PointCloud2 主题
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scanp", qos,
            std::bind(&PointCloudToLaserScanNode::pointcloud_callback, this, std::placeholders::_1));

        // 发布 LaserScan 主题
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", qos);

        // 参数设置（可以根据需要调整）
        this->declare_parameter<double>("angle_min", -3.14); // -90 degrees
        this->declare_parameter<double>("angle_max", 3.14);  // 90 degrees
        this->declare_parameter<double>("angle_increment", 0.01); // 0.01 rad
        this->declare_parameter<double>("range_min", 0.0);
        this->declare_parameter<double>("range_max", 10.0);
        this->declare_parameter<std::string>("target_frame", "base_footprint"); // 默认目标帧
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 获取目标帧
        std::string target_frame = this->get_parameter("target_frame").as_string();

        // 尝试进行坐标变换
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try
        {
            // 等待变换的可用性
            if (!tf_buffer_.canTransform(target_frame, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)))
            {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s", msg->header.frame_id.c_str(), target_frame.c_str());
                return;
            }

            // 执行坐标变换
            tf2::doTransform(*msg, transformed_cloud, tf_buffer_.lookupTransform(target_frame, msg->header.frame_id, msg->header.stamp));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        // 转换 PointCloud2 到 PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

        // 获取参数
        double angle_min = this->get_parameter("angle_min").as_double();
        double angle_max = this->get_parameter("angle_max").as_double();
        double angle_increment = this->get_parameter("angle_increment").as_double();
        double range_min = this->get_parameter("range_min").as_double();
        double range_max = this->get_parameter("range_max").as_double();

        // 计算激光扫描的角度数量
        int num_readings = std::ceil((angle_max - angle_min) / angle_increment);

        // 初始化 LaserScan 消息
        sensor_msgs::msg::LaserScan scan;
        scan.header = transformed_cloud.header; // 使用变换后的点云头部
        scan.angle_min = angle_min;
        scan.angle_max = angle_max;
        scan.angle_increment = angle_increment;
        scan.time_increment = 0.0;
        scan.scan_time = this->get_clock()->now().seconds(); // 可以根据需要调整
        scan.range_min = range_min;
        scan.range_max = range_max;
        scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

        // 将点云转换为 LaserScan
        for (const auto& point : pcl_cloud->points)
        {
            // 计算极坐标
            double angle = std::atan2(point.y, point.x);
            double range = std::sqrt(point.x * point.x + point.y * point.y);

            // 检查范围和角度是否在有效范围内
            if (range < range_min || range > range_max)
                continue;
            if (angle < angle_min || angle > angle_max)
                continue;

            // 计算对应的索引
            int index = static_cast<int>((angle - angle_min) / angle_increment);
            if (index >= 0 && index < num_readings)
            {
                // 更新最小距离
                if (range < scan.ranges[index])
                {
                    scan.ranges[index] = range;
                }
            }
        }

        // 发布 LaserScan 消息
        laserscan_publisher_->publish(scan);

        RCLCPP_INFO(this->get_logger(), "Receiving PointCloud from frame: %s", msg->header.frame_id.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;

    // TF2 成员
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToLaserScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
