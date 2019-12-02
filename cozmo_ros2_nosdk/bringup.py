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
    BatteryState,
    Image,
    Imu,
    JointState,
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
        self._odom_frame             = "/map"
        self._base_frame             = "/base_link"
        self._bridge                 = CvBridge()
        self._last_pose              = Pose()
        self._imu_msg                = Imu(header = Header(frame_id = self._base_frame))
        self._odom_msg               = Odometry(header = Header(frame_id = self._odom_frame),
                                                child_frame_id = self._base_frame,
                                                pose = PoseWithCovariance())
        self._js_msg                 = JointState()
        self._js_msg.header.frame_id = self._base_frame
        self._js_msg.name            = ['head', 'lift']
        self._js_msg.velocity        = [0.0, 0.0]
        self._js_msg.effort          = [0.0, 0.0]
        self._battery_msg            = BatteryState()
        self._battery_msg.present    = True
        # self._tf_msg     = TransformStamped(header = Header(frame_id = self._odom_frame),
        #                                    child_frame_id = self._base_frame)

        # Subscribers
        self._twist_sub = self.node.create_subscription(Twist, 'cmd_vel', self._control, 10)

        # Publishers
        self._img_pub         = self.node.create_publisher(Image, 'camera', 1)
        self._imu_pub         = self.node.create_publisher(Imu, 'imu', 1)
        self._odom_pub        = self.node.create_publisher(Odometry, "odom", 1)
        self._joint_state_pub = self.node.create_publisher(JointState, 'joints', 1)
        self._battery_pub     = self.node.create_publisher(BatteryState, 'battery', 1)
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
        self._publish_joint_state(now)
        self._publish_battery(now)

    def _pub_odom(self, now):
        """
        Publish imu data as Imu.

        """
        # Publish only if there are subscribers
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
        # Publish only if there are subscribers
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

    def _publish_joint_state(self, now):
        """
        Publish joint states as JointStates.

        """
        # Publish only if there are subscribers
        if self._joint_state_pub.get_subscription_count() == 0:
            return

        self._js_msg.header.stamp = now
        self._js_msg.position     = [self.head_angle.radians,
                                     self.lift_position * 0.001]
        self._joint_state_pub.publish(self._js_msg)

    def _publish_battery(self, now):
        """
        Publish battery as BatteryState message.

        """
        # Publish only if there are subscribers
        if self._battery_pub.get_subscription_count() == 0:
            return

        self._battery_msg.header.stamp = now
        self._battery_msg.voltage      = self.battery_voltage
        self._battery_pub.publish(self._battery_msg)

    def _robot_charging(self, cli, state):
        """
        Publish battery as BatteryState message.

        """
        del cli

        if self._battery_pub.get_subscription_count() == 0:
            return

        now = self.node.get_clock().now().to_msg()
        self._battery_msg.header.stamp = now
        if state:
            self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        self._battery_pub.publish(self._battery_msg)

    def _cliff_detection_cb(self, cli, state):
        del cli
        if state:
            self.node.get_logger().info("Cliff detected.")

    def _robot_kidnapping_cb(self, cli, state):
        del cli
        if state:
            self.node.get_logger().info("Picked up.")
        else:
            self.node.get_logger().info("Put down.")

    def _robot_poking_cb(self, cli, pkt):
        # pkt : pycozmo.protocol_encoder.RobotPoked
        del cli, pkt
        self.node.get_logger().info("Robot poked.")

    def _robot_falling_cb(self, cli, pkt):
        del cli
        if type(pkt) == protocol_encoder.FallingStarted:
            self.node.get_logger().info("Started falling.")
        elif type(pkt) == protocol_encoder.FallingStopped:
            self.node.get_logger().info("Falling stopped after {} ms. Impact intensity {:.01f}.".format(pkt.duration_ms, pkt.impact_intensity))
        else:
            pass

    def _button_pressed_cb(self, cli, pkt):
        del cli
        if pkt.pressed:
            self.node.get_logger().info("Button pressed.")
        else:
            self.node.get_logger().info("Button released.")

    def _robot_wheels_moving_cb(self, cli, state):
        del cli
        if state:
            self.node.get_logger().info("Started moving.")
        else:
            self.node.get_logger().info("Stopped moving.")


    def _image_cb(self, cli, img):
        """
        Publish camera image as Image.

        """
        del cli

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
        self.add_handler(event.EvtRobotStateUpdated,           self._robot_state_cb)
        self.add_handler(event.EvtNewRawCameraImage,           self._image_cb)
        self.add_handler(event.EvtRobotChargingChange,         self._robot_charging)
        self.add_handler(event.EvtRobotPickedUpChange,         self._robot_kidnapping_cb)
        self.add_handler(event.EvtCliffDetectedChange,         self._cliff_detection_cb)
        self.add_handler(event.EvtRobotWheelsMovingChange,     self._robot_wheels_moving_cb)
        self.conn.add_handler(protocol_encoder.RobotPoked,     self._robot_poking_cb)
        self.conn.add_handler(protocol_encoder.FallingStarted, self._robot_falling_cb)
        self.conn.add_handler(protocol_encoder.FallingStopped, self._robot_falling_cb)
        self.conn.add_handler(protocol_encoder.ButtonPressed,  self._button_pressed_cb)

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