#include <memory>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>  // 使用新的头文件
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  // 使用新的头文件

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// 下面的代码保持不变
class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger()
    : Node("pointcloud_merger"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        rclcpp::QoS qos{rclcpp::SensorDataQoS()};

        sub_front_.subscribe(this, "/scan_front", qos.get_rmw_qos_profile());
        sub_rear_.subscribe(this, "/scan_rear", qos.get_rmw_qos_profile());

        sync_.reset(new Sync(MySyncPolicy(5), sub_front_, sub_rear_));
        sync_->registerCallback(std::bind(&PointCloudMerger::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scanp", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloudMerger node has been started.");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_front_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_rear_;
    std::shared_ptr<Sync> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::mutex mutex_;

    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr front_msg,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr rear_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*front_msg, *cloud_front);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rear(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*rear_msg, *cloud_rear);

        try {
            // Transform front cloud to base_link frame using actual message time
            // Use longer timeout (50ms) for TF lookups to handle 50Hz sensor rate
            geometry_msgs::msg::TransformStamped transform_front = tf_buffer_.lookupTransform(
                "odom", front_msg->header.frame_id, front_msg->header.stamp, 
                tf2::durationFromSec(0.2)); // Increased timeout to 200ms

            Eigen::Isometry3f eigen_transform_front = tf2::transformToEigen(transform_front.transform).cast<float>();
            pcl::transformPointCloud(*cloud_front, *cloud_front, eigen_transform_front);

            // Transform rear cloud to base_link frame using actual message time
            geometry_msgs::msg::TransformStamped transform_rear = tf_buffer_.lookupTransform(
                "odom", rear_msg->header.frame_id, rear_msg->header.stamp,
                tf2::durationFromSec(0.2)); // Increased timeout to 200ms

            Eigen::Isometry3f eigen_transform_rear = tf2::transformToEigen(transform_rear.transform).cast<float>();
            pcl::transformPointCloud(*cloud_rear, *cloud_rear, eigen_transform_rear);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed transform: %s", ex.what());
            return;
        }

        // Merge point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *merged_cloud = *cloud_front + *cloud_rear;

        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(*merged_cloud, merged_msg);

        // Compare timestamps using rclcpp::Time
        rclcpp::Time front_time(front_msg->header.stamp);
        rclcpp::Time rear_time(rear_msg->header.stamp);
        // Check timestamp synchronization quality
        auto time_diff = std::abs((front_time - rear_time).seconds());
        if (time_diff > 0.01) {  // 10ms tolerance
            RCLCPP_WARN(this->get_logger(), 
                "Large time difference between scans: %.3f ms", time_diff*1000);
        }
        
        // Use newest timestamp to minimize latency
        merged_msg.header.stamp = (front_time < rear_time) ? front_msg->header.stamp : rear_msg->header.stamp;
        merged_msg.header.frame_id = "odom";

        publisher_->publish(merged_msg);

        RCLCPP_INFO(this->get_logger(), "Published merged point cloud with %zu points.", merged_cloud->points.size());
    }

    builtin_interfaces::msg::Time average_time(const builtin_interfaces::msg::Time &t1, const builtin_interfaces::msg::Time &t2)
    {
        builtin_interfaces::msg::Time avg;

        uint64_t total_sec = static_cast<uint64_t>((t1.sec + t2.sec) / 2);
        uint64_t total_nanosec = static_cast<uint64_t>((t1.nanosec + t2.nanosec) / 2);

        if (total_nanosec >= 1000000000)
        {
            total_sec += 1;
            total_nanosec -= 1000000000;
        }

        avg.sec = static_cast<int32_t>(total_sec);
        avg.nanosec = static_cast<uint32_t>(total_nanosec);

        return avg;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
