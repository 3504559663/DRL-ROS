#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from nav_msgs.msg import Odometry
import math

class TFStabilizer(Node):
    def __init__(self):
        super().__init__('tf_stabilizer')
        
        # 声明并获取参数
        self.declare_parameter('update_frequency', 50.0)
        self.declare_parameter('smoothing_factor', 0.8)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        self.update_frequency = self.get_parameter('update_frequency').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # 创建TF监听器和广播器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建里程计订阅器
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/bot',  # Gazebo Ackermann插件发布的里程计话题
            self.odom_callback,
            10)
        
        # 存储上一次的变换
        self.last_transform = None
        
        # 状态变量
        self.is_rotating = False
        self.last_yaw = 0.0
        self.yaw_change_sum = 0.0
        self.rotation_threshold = 0.1  # 10度每秒
        
        # 平滑滤波器状态
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_yaw = 0.0
        self.is_filter_initialized = False
        
        # 创建定时器
        self.timer = self.create_timer(1.0/self.update_frequency, self.update_tf)
        
        self.get_logger().info('TF稳定器已启动')
    
    def odom_callback(self, msg):
        # 从里程计提取角速度，检测是否正在旋转
        angular_velocity = msg.twist.twist.angular.z
        self.is_rotating = abs(angular_velocity) > self.rotation_threshold
        
        # 计算yaw角变化
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # 四元数转yaw角
        current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        if self.last_yaw is not None:
            yaw_diff = self.normalize_angle(current_yaw - self.last_yaw)
            self.yaw_change_sum += abs(yaw_diff)
        
        self.last_yaw = current_yaw
        
        # 记录大角度旋转
        if self.yaw_change_sum > math.pi/4:  # 每累计45度记录一次
            self.get_logger().info(f'检测到大角度旋转: {self.yaw_change_sum:.2f} rad')
            self.yaw_change_sum = 0.0
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def update_tf(self):
        try:
            # 获取map→odom变换
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time())
            
            # 提取变换参数
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            # 计算yaw角
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # 初始化滤波器
            if not self.is_filter_initialized:
                self.filtered_x = x
                self.filtered_y = y
                self.filtered_yaw = yaw
                self.is_filter_initialized = True
            
            # 应用滤波器 - 旋转时使用更高的平滑因子
            smoothing = self.smoothing_factor
            if self.is_rotating:
                smoothing = 0.95  # 旋转时更平滑
                self.get_logger().debug('正在旋转，增强平滑因子')
            
            self.filtered_x = smoothing * self.filtered_x + (1.0 - smoothing) * x
            self.filtered_y = smoothing * self.filtered_y + (1.0 - smoothing) * y
            
            # 角度平滑需要特殊处理
            yaw_diff = self.normalize_angle(yaw - self.filtered_yaw)
            self.filtered_yaw = self.normalize_angle(self.filtered_yaw + (1.0 - smoothing) * yaw_diff)
            
            # 从平滑后的yaw创建四元数
            quat = self.euler_to_quaternion(0.0, 0.0, self.filtered_yaw)
            
            # 创建平滑的变换
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            t.transform.translation.x = self.filtered_x
            t.transform.translation.y = self.filtered_y
            t.transform.translation.z = z  # z保持不变
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            # 广播平滑的变换
            self.tf_broadcaster.sendTransform(t)
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取变换: {ex}')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TFStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
