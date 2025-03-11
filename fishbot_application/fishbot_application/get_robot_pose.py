import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener,Buffer
from tf_transformations import euler_from_quaternion
import math #angle to radian

class Listener(Node):
    def __init__(self):
        super().__init__('tf_broadcater')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.timer = self.create_timer(1, self.publish_tf)
    def publish_tf(self):
        try:
            result = self.buffer.lookup_transform('map', 'base_footprint',
                                                  rclpy.time.Time(seconds = 0.0),rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f'平移:{transform.translation}')
            self.get_logger().info(f'旋转:{transform.rotation}')
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.transform_euler = (f'ROTATION PPT : {rotation_euler}')


        except Exception as e:
            self.get_logger().info(f'ABNORMAL HAPPEND, REASON:{str(e)}')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()