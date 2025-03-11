import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener,Buffer
from tf_transformations import euler_from_quaternion,quaternion_from_euler
from rclpy.duration import Duration
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    def __init__(self, node_name = 'patrol_node'):
        super().__init__(node_name)
        self.declare_parameter('initial_point', [0.0,0.0,0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

        self.declare_parameter('image_save_path','/home/alan/chapt7/chapt7_ws/img')
        self.image_save_path_ = self.get_parameter('image_save_path').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.sub = self.create_subscription(Image,'/camera_sensor/image_raw',self.image_callback,1)


    def image_callback(self,msg):
        self.latest_image = msg
    
    def record_image(self):
        if self.latest_image is not None:
            pose = self.get_current_pose()
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image)
            path=f'{self.image_save_path_}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png'
            cv2.imwrite(path,cv_image)



    def get_pose_by_xyyaw(self,x,y,yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        #pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def init_robot_pose(self):
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0],self.initial_point_[1],self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()



    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for i in range(int(len(self.target_points_)/3)):
            x = self.target_points_[i*3]
            y = self.target_points_[i*3+1]
            yaw = self.target_points_[i*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'目标点{i}: {x},{y},{yaw}')
        return points

    def nav_to_pose(self,target_pose):
        self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f'预计剩余距离: {feedback.distance_remaining}')
        result = self.getResult()
        self.get_logger().info(f'导航结果: {result}')

    def get_current_pose(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        while rclpy.ok():
            try:
                result = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                    rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
                transform = result.transform
                self.get_logger().info(f'平移:{transform.translation}')
                return transform
            except Exception as e:
                self.get_logger().info(f'ABNORMAL HAPPEND, REASON:{str(e)}')


def main():
    rclpy.init()
    patrol = PatrolNode()

    patrol.init_robot_pose()

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x,y,yaw = point[0],point[1],point[2]
            target_pose = patrol.get_pose_by_xyyaw(x,y,yaw)
            patrol.nav_to_pose(target_pose)

            patrol.record_image()
            patrol.get_logger().info('图像保存成功')
    rclpy.spin(patrol)
    rclpy.shutdown()