import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import copy
from geometry_msgs.msg import (
    TwistWithCovariance,
    PoseWithCovarianceStamped,
    PoseWithCovariance,
    Pose,
    PoseStamped,
    Twist,
    Transform,
    TransformStamped,
    Quaternion,
)

from autoware_auto_vehicle_msgs.msg import VelocityReport
from carla_msgs.msg import CarlaEgoVehicleInfo,CarlaEgoVehicleStatus
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

import math
import csv
import os
import yaml


class VelocityReceiver(Node):
    def __init__(self):
        super().__init__("velocity_receiver")

        config_path = config_path = os.path.dirname(os.path.realpath(__file__))+"/../config/" + "config.yaml"

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        tf_static_path = config["tf_static_yaml"]
        initialpose_path = config["initialpose_yaml"]
        groundtruth_path = config["groundtruth_path"]

        self.carla_status_subscription = self.create_subscription(
            CarlaEgoVehicleStatus,
            "/carla/ego_vehicle/vehicle_status",
            self.carla_status_listener_callback,
            1,
        )

        self.odometry_subscription = self.create_subscription(
            Odometry,
            "/carla/ego_vehicle/odometry",
            self.odometry_listener_callback,
            1,
        )

        self.velocity_publisher = self.create_publisher(
            VelocityReport,
            "/vehicle/status/velocity_status",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        self.groundtruth_publisher = self.create_publisher(
            PoseStamped,
            "/groundtruth_pose",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        self.tf_static_publisher = self.create_publisher(
            TFMessage,
            "/tf_static",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose_temp",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )



        self.vehicle_status = None
        self.vehicle_info = None
        self.stamped_time = None
        self.out_report = None

        self.load_initialpose(initialpose_path)
        self.load_tf_static(tf_static_path)

        csv_file = open(groundtruth_path, 'w', newline='')
        self.csv_writer = csv.writer(csv_file)
        self.csv_writer.writerow(["sec", "nsec", "x", "y"])

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_tf_static(self, tf_static_path):
        with open(tf_static_path, 'r') as f:
            tf_static_data = yaml.safe_load(f)

        tf_msg = TFMessage()
        static_transforms = []

        for transform in tf_static_data['static_transforms']:
            tf = TransformStamped()
            tf.header.frame_id = transform['header']['frame_id']
            tf.child_frame_id = transform['child_frame_id']
            
            tf.transform.translation.x = transform['transform']['translation']['x']
            tf.transform.translation.y = transform['transform']['translation']['y']
            tf.transform.translation.z = transform['transform']['translation']['z']

            tf.transform.rotation.x = transform['transform']['rotation']['x']
            tf.transform.rotation.y = transform['transform']['rotation']['y']
            tf.transform.rotation.z = transform['transform']['rotation']['z']
            tf.transform.rotation.w = transform['transform']['rotation']['w']

            static_transforms.append(tf)
        
        tf_msg.transforms = static_transforms
        self.tf_static_publisher.publish(tf_msg)

    def load_initialpose(self, initialpose_path):
        with open(initialpose_path, 'r') as f:
            initialpose_data = yaml.safe_load(f)

        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.header.frame_id = "map"
        position = initialpose_data["position"]
        orientation = initialpose_data["orientation"]

        initialpose_msg.pose.pose.position.x = position["x"]
        initialpose_msg.pose.pose.position.y = position["y"]
        initialpose_msg.pose.pose.position.z= position["z"]

        initialpose_msg.pose.pose.orientation.x = orientation["x"]
        initialpose_msg.pose.pose.orientation.y = orientation["y"]
        initialpose_msg.pose.pose.orientation.z = orientation["z"]
        initialpose_msg.pose.pose.orientation.w= orientation["w"]

        self.initialpose_publisher.publish(initialpose_msg)

    def odometry_listener_callback(self,msg):
        groundtruth_msg = PoseStamped()
        groundtruth_msg.header = msg.header
        groundtruth_msg.pose = msg.pose.pose
        groundtruth_msg.header.frame_id = "map"
        groundtruth_publisher.publish(groundtruth_msg)
        self.csv_writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.pose.position.x, msg.pose.pose.position.y])


    def carla_status_listener_callback(self,msg):
        self.get_logger().info("try to catch vehicle status message from carla")
        self.vehicle_status = msg
        self.update_vehicle_report()

    def timer_callback(self):
        if self.out_report is not None:
            self.velocity_publisher.publish(self.out_report)

    def update_vehicle_report(self):
        veh_report = VelocityReport()
        veh_report.longitudinal_velocity = self.vehicle_status.velocity
        veh_report.lateral_velocity = 0.0
        self.out_report = veh_report

    

def main():
    rclpy.init()
    velocity_receiver = VelocityReceiver()
    rclpy.spin(velocity_receiver)
    velocity_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
