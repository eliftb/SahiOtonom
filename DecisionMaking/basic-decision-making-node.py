#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive
from enum import Enum
import numpy as np
import time
from utils.ros_logger import apply_log_level



class DecisionMakingNode(Node):
    """
    Basit karar verme node'u: Düz yolda şerit takibi ve engel önleme
    """
    def __init__(self):
        super().__init__('decision_making_node')
        
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('steering_gain', 1.5)
        self.declare_parameter('emergency_stop_distance', 1.5)
        self.declare_parameter('slow_down_distance', 4.0)
        self.declare_parameter('obstacle_slowdown_enabled', True)
        self.declare_parameter('barrier_correction_gain', 1.0)
        self.declare_parameter('max_barrier_correction', 0.35)
        self.declare_parameter('barrier_slow_distance', 1.0)
        self.declare_parameter('barrier_min_speed_ratio', 0.65)
        self.declare_parameter('recovery_enabled', True)
        self.declare_parameter('recovery_speed', -0.08)
        self.declare_parameter('recovery_duration', 1.2)
        self.declare_parameter('recovery_steering_angle', 0.30)
        
        self.BASE_SPEED = float(self.get_parameter('base_speed').value)
        self.MAX_STEERING_ANGLE = float(self.get_parameter('max_steering_angle').value)
        self.STEERING_GAIN = float(self.get_parameter('steering_gain').value)
        self.EMERGENCY_STOP_DISTANCE = float(self.get_parameter('emergency_stop_distance').value)
        self.SLOW_DOWN_DISTANCE = float(self.get_parameter('slow_down_distance').value)
        self.OBSTACLE_SLOWDOWN_ENABLED = bool(self.get_parameter('obstacle_slowdown_enabled').value)
        self.BARRIER_CORRECTION_GAIN = float(self.get_parameter('barrier_correction_gain').value)
        self.MAX_BARRIER_CORRECTION = float(self.get_parameter('max_barrier_correction').value)
        self.BARRIER_SLOW_DISTANCE = float(self.get_parameter('barrier_slow_distance').value)
        self.BARRIER_MIN_SPEED_RATIO = float(self.get_parameter('barrier_min_speed_ratio').value)
        self.RECOVERY_ENABLED = bool(self.get_parameter('recovery_enabled').value)
        self.RECOVERY_SPEED = float(self.get_parameter('recovery_speed').value)
        self.RECOVERY_DURATION = float(self.get_parameter('recovery_duration').value)
        self.RECOVERY_STEERING_ANGLE = float(self.get_parameter('recovery_steering_angle').value)
        
        self.manual_speed = self.BASE_SPEED
        self.lateral_deviation = 0.0
        self.barrier_lateral_correction = 0.0
        self.barrier_min_distance = float('inf')
        self.barrier_safety_stop = False
        self.recovery_until = 0.0
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')


        self.lateral_sub = self.create_subscription(
            Float32, '/lane/lateral_deviation', self.lateral_callback, 10)
        
        self.obstacle_detected_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_detected_callback, 10)
        
        self.obstacle_distance_sub = self.create_subscription(
            Float32, '/obstacle_distance', self.obstacle_distance_callback, 10)

        self.barrier_correction_sub = self.create_subscription(
            Float32, '/barrier/lateral_correction', self.barrier_correction_callback, 10)
        self.barrier_min_distance_sub = self.create_subscription(
            Float32, '/barrier/min_distance', self.barrier_min_distance_callback, 10)
        self.barrier_safety_stop_sub = self.create_subscription(
            Bool, '/barrier/safety_stop', self.barrier_safety_stop_callback, 10)
        

        self.ackermann_pub = self.create_publisher(
            AckermannDrive, '/ackermann_cmd', 10)
        
        self.speed_pub = self.create_publisher(
            Float32, '/speed', 10)

        self.timer = self.create_timer(0.1, self.decision_loop)  # 10 Hz
        
        self.get_logger().info('🚗 Decision Making Node başlatıldı.')
        apply_log_level(self)

    def lateral_callback(self, msg):
        """Şerit sapma bilgisini al"""
        self.lateral_deviation = msg.data

    def obstacle_detected_callback(self, msg):
        """Engel tespit durumunu al"""
        self.obstacle_detected = msg.data

    def obstacle_distance_callback(self, msg):
        """Engel mesafesini al"""
        if msg.data > 0:
            self.obstacle_distance = msg.data
        else:
            self.obstacle_distance = float('inf')

    def barrier_correction_callback(self, msg):
        """Bariyerlerden gelen yan kaçınma düzeltmesini al."""
        correction = float(msg.data) * self.BARRIER_CORRECTION_GAIN
        self.barrier_lateral_correction = float(
            np.clip(correction, -self.MAX_BARRIER_CORRECTION, self.MAX_BARRIER_CORRECTION)
        )

    def barrier_min_distance_callback(self, msg):
        if msg.data > 0:
            self.barrier_min_distance = float(msg.data)
        else:
            self.barrier_min_distance = float('inf')

    def barrier_safety_stop_callback(self, msg):
        self.barrier_safety_stop = bool(msg.data)

    def calculate_steering_angle(self):
        """Direksiyon açısını hesapla - Düz yol için basit şerit takibi"""
        if self.is_recovering():
            if self.barrier_lateral_correction != 0.0:
                recovery_steer = -np.sign(self.barrier_lateral_correction)
            elif self.lateral_deviation != 0.0:
                recovery_steer = -np.sign(self.lateral_deviation)
            else:
                recovery_steer = -1.0
            return float(recovery_steer * self.RECOVERY_STEERING_ANGLE)

        corrected_deviation = self.lateral_deviation + self.barrier_lateral_correction
        steering_angle = -corrected_deviation * self.STEERING_GAIN
        
        # Maksimum direksiyon açısı sınırlaması
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        
        return steering_angle

    def is_recovering(self):
        return self.RECOVERY_ENABLED and time.time() < self.recovery_until

    def calculate_speed(self):
        """Hız hesapla - mesafeye göre yavaşlama ve acil duruş"""
        if self.barrier_safety_stop and self.RECOVERY_ENABLED:
            self.recovery_until = max(self.recovery_until, time.time() + self.RECOVERY_DURATION)

        if self.is_recovering():
            self.get_logger().warn('↩️  Bariyer kurtarma: kısa geri kaçış.', throttle_duration_sec=1.0)
            return self.RECOVERY_SPEED

        if self.barrier_safety_stop:
            self.get_logger().warn('⛔ Bariyer güvenlik duruşu!', throttle_duration_sec=1.0)
            return 0.0

        speed = self.manual_speed

        if self.barrier_min_distance <= self.BARRIER_SLOW_DISTANCE:
            ratio = max(self.BARRIER_MIN_SPEED_RATIO, self.barrier_min_distance / self.BARRIER_SLOW_DISTANCE)
            speed *= ratio

        if self.obstacle_detected:
            if self.obstacle_distance <= self.EMERGENCY_STOP_DISTANCE:
                speed = 0.0
                self.get_logger().warn(
                    f'⛔ Acil duruş! Engel mesafesi: {self.obstacle_distance:.2f}m')
            elif self.OBSTACLE_SLOWDOWN_ENABLED and self.obstacle_distance <= self.SLOW_DOWN_DISTANCE:
                distance_window = self.SLOW_DOWN_DISTANCE - self.EMERGENCY_STOP_DISTANCE
                if distance_window <= 0.0:
                    speed = 0.0
                else:
                    ratio = (self.obstacle_distance - self.EMERGENCY_STOP_DISTANCE) / distance_window
                    speed = self.manual_speed * max(0.0, ratio)
                self.get_logger().info(
                    f'⚠️  Yavaşlama! Engel mesafesi: {self.obstacle_distance:.2f}m | Hız: {speed:.2f}',
                    throttle_duration_sec=1.0)

        return speed

    def decision_loop(self):
        """Ana karar verme döngüsü"""
        try:
            # Direksiyon açısını hesapla
            steering_angle = self.calculate_steering_angle()
            
            # Hızı hesapla
            speed = self.calculate_speed()
            
            # Ackermann mesajı hazırla ve yayınla
            ackermann_msg = AckermannDrive()
            ackermann_msg.speed = speed
            ackermann_msg.steering_angle = steering_angle
            ackermann_msg.steering_angle_velocity = 0.0
            ackermann_msg.acceleration = 0.0
            ackermann_msg.jerk = 0.0
            
            self.ackermann_pub.publish(ackermann_msg)
            
            # Speed mesajı hazırla ve yayınla
            speed_msg = Float32()
            speed_msg.data = speed
            self.speed_pub.publish(speed_msg)
            
            self.get_logger().debug(
                f'Speed: {speed:.2f} m/s | Steering: {steering_angle:.3f} rad | '
                f'Deviation: {self.lateral_deviation:.3f} '
                f'BarrierCorr: {self.barrier_lateral_correction:.3f} | '
                f'Obstacle: {self.obstacle_detected} | '
                f'Distance: {self.obstacle_distance:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Decision loop error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
