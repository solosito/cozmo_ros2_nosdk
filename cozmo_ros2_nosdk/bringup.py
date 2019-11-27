# General
import numpy as np
import time
from copy import deepcopy

# OpenCV
import cv2
from cv_bridge import CvBridge

# Pycozmo
import pycozmo
from pycozmo import (
    Client,
    event,
    protocol_encoder,
)

# ROS2
import rclpy
from geometry_msgs.msg import (
   Pose,
   PoseWithCovariance,
   Quaternion,
   Twist,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (
    Image,
    Imu,
)
from std_msgs.msg import Header


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def quaternion_to_euler(q):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(t3, t4)
    return roll, pitch, yaw


class Cozmo(Client):
    def __init__(self, node_name, namespace=None):
        super().__init__()
        self._robot_started = False
        self._ros2_started  = False
        self._init_robot()
        self._init_ros2(node_name, namespace)
        self._start_camera()
        rclpy.spin(self.node)

    def _start_camera(self):
        pkt = protocol_encoder.EnableCamera(enable=True)
        self.conn.send(pkt)
        pkt = protocol_encoder.EnableColorImages(enable=True)
        self.conn.send(pkt)
        time.sleep(2.0) # Wait for image to stabilize.

    def __del__(self):
        if self._ros2_started:
            self.node.destroy_node()

    def _init_ros2(self, node_name, namespace):
        self.node = rclpy.create_node(node_name=node_name,
                                      namespace=namespace or node_name)

        # Variables
        self._odom_frame = "/map"
        self._base_frame = "/base_link"
        self._bridge     = CvBridge()
        self._last_pose  = Pose()
        self._imu_msg    = Imu(header = Header(frame_id = self._base_frame))
        self._odom_msg   = Odometry(header = Header(frame_id = self._odom_frame),
                                    child_frame_id = self._base_frame,
                                    pose = PoseWithCovariance())
        # self._tf_msg     = TransformStamped(header = Header(frame_id = self._odom_frame),
        #                                    child_frame_id = self._base_frame)

        # Subscribers
        self._twist_sub = self.node.create_subscription(Twist, 'cmd_vel', self._control, 10)

        # Publishers
        self._img_pub  = self.node.create_publisher(Image, 'camera', 1)
        self._imu_pub  = self.node.create_publisher(Imu, 'imu', 1)
        self._odom_pub = self.node.create_publisher(Odometry, "odom", 1)
        # self._tf_pub   = self.create_publisher(TFMessage, "tf", 10)

        self._ros2_started = True

    def _init_robot(self):
        super().start()
        self._start()
        self.connect()
        self.wait_for_robot()
        self._robot_started = True

    def _robot_state_cb(self, cli):
        del cli

        # Updated data on EvtRobotStateUpdated:
        #
        #   timestamp, pose_frame_id, pose_origin_id, pose_x, pose_y, pose_z, pose_angle_rad,
        #   pose_pitch_rad, lwheel_speed_mmps, rwheel_speed_mmps, head_angle_rad, lift_height_mm
        #   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, battery_voltage, status,
        #   cliff_data_raw, backpack_touch_sensor_raw, curr_path_segment

        if not self._ros2_started:
            return

        now = self.node.get_clock().now().to_msg()

        self._pub_imu(now)
        self._pub_odom(now)

    def _pub_odom(self, now):
        """
        Publish imu data as Imu.

        """
        # Publish only ros if there are subscribers:
        if self._odom_pub.get_subscription_count() == 0:
            return

        self._odom_msg.header.stamp          = now
        self._odom_msg.pose.pose.position.x  = self.pose.x * 0.001 # Units?
        self._odom_msg.pose.pose.position.y  = self.pose.y * 0.001 # Units?
        self._odom_msg.pose.pose.position.z  = self.pose.z * 0.001 # Units?
        self._odom_msg.pose.pose.orientation = euler_to_quaternion(0.0, self.pose_pitch.radians, self.pose_angle.radians)
        self._odom_msg.pose.covariance       = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        # self._odom_msg.twist.twist.linear.x  = self._lin_vel
        # self._odom_msg.twist.twist.angular.z = self._ang_vel
        self._odom_msg.twist.covariance      = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()

        self._odom_pub.publish(self._odom_msg)

        self._last_pose = deepcopy(self._odom_msg.pose.pose)

    def _pub_imu(self, now):
        """
        Publish imu data as Imu.

        """
        # Publish only ros if there are subscribers:
        if self._imu_pub.get_subscription_count() == 0:
            return

        self._imu_msg.header.stamp          = now
        self._imu_msg.orientation           = euler_to_quaternion(0.0, 0.0, self.pose_angle)
        self._imu_msg.linear_acceleration.x = self.accel.x * 0.001 # Units?
        self._imu_msg.linear_acceleration.y = self.accel.y * 0.001 # Units?
        self._imu_msg.linear_acceleration.z = self.accel.z * 0.001 # Units?
        self._imu_msg.angular_velocity.x    = self.gyro.x # Units?
        self._imu_msg.angular_velocity.y    = self.gyro.y # Units?
        self._imu_msg.angular_velocity.z    = self.gyro.z # Units?

        self._imu_pub.publish(self._imu_msg)

    def _image_cb(self, cli, img):
        """
        Publish camera image as Image.

        """
        # Publish only if ROS2 is started and there are subscribers
        if not self._ros2_started or self._img_pub.get_subscription_count() == 0:
            return

        now = self.node.get_clock().now().to_msg()
        cv_image = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        img_msg = self._bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera"

        self._img_pub.publish(img_msg)

    def _start(self):
        self.add_handler(event.EvtRobotStateUpdated, self._robot_state_cb)
        self.add_handler(event.EvtNewRawCameraImage, self._image_cb)

    def _control(self, twist_msg):
        """
        Set commanded velocities from Twist message.

        :type   twist_msg:    Twist
        :param  twist_msg:    The commanded velocities.

        """

        left_wheel  = (twist_msg.linear.x - twist_msg.angular.z) * pycozmo.MAX_WHEEL_SPEED.mmps
        right_wheel = (twist_msg.linear.x + twist_msg.angular.z) * pycozmo.MAX_WHEEL_SPEED.mmps

        self.drive_wheels(lwheel_speed=left_wheel, rwheel_speed=right_wheel)
        self.move_head(twist_msg.linear.y)
        self.move_lift(twist_msg.linear.z)

def main(args=None):
    rclpy.init(args=args)
    cozmo_node = Cozmo(node_name="cozmo")
    cozmo_node.start()
    rclpy.shutdown()

if __name__=="__main__":
    main()