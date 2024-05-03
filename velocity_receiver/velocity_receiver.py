import rclpy
from rclpy.node import Node
import copy
from geometry_msgs.msg import (
    TwistWithCovariance,
    PoseWithCovarianceStamped,
    PoseWithCovariance,
    Pose,
    Twist,
    Transform,
    TransformStamped,
    Quaternion,
)

from autoware_auto_vehicle_msgs.msg import VelocityReport
from carla_msgs.msg import CarlaEgoVehicleInfo,CarlaEgoVehicleStatus
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy
import math


class VelocityReceiver(Node):
    def __init__(self):
        super().__init__("velocity_receiver")
        self.declare_parameter("angle", rclpy.Parameter.Type.DOUBLE)

        self.carla_info_subscription = self.create_subscription(
            CarlaEgoVehicleInfo,
            "/carla/ego_vehicle/vehicle_info",
            self.carla_listener_callback,
            1,
        )
        self.carla_status_subscription = self.create_subscription(
            CarlaEgoVehicleStatus,
            "/carla/ego_vehicle/vehicle_status",
            self.carla_listener_callback,
            1,
        )

        self.init_publisher = self.create_publisher(
            VelocityReport,
            "/vehicle/status/velocity_status",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        self.vehicle_status = None
        self.vehicle_info = None
        self.steer = 0.0
        self.stamped_time = None
        self.out_report = None
        

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def carla_info_listener_callback(self, msg):
        self.get_logger().info("try to catch vehicle info message from carla")
        self.vehicle_info = msg
        self.update_vehicle_report()
    def carla_status_listener_callbach(self,msg):
        self.get_logger().info("try to catch vehicle status message from carla")
        self.vehicle_status = msg
        self.update_vehicle_report()

    def timer_callback(self):
        if self.out_report is not None:
            self.init_publisher.publish(self.out_report)

    def update_vehicle_report(self):
        if self.vehicle_status is None or self.vehicle_info is None:
            return
        veh_report = VelocityReport()
        veh_report.longitudinal_velocity = self.vehicle_status.velocity
        veh_report.lateral_velocity = 0.0
        veh_report.heading_rate = self.compute_heading()
        self.out_report = veh_report

    def compute_heading(self):
        time_s = self.vehicle_status.control.header.stamp.secs
        time_ns = self.vehicle_status.control.header.stamp.nsecs
        time = time_s + time_ns*(10**(-9))
        if self.stamped_time is None:
            self.stamped_time = time
            return 0.0
        
        max_angle = self.vehicle_info.wheels.max_steer_angle
        steering = self.vehicle_status.control.steer
        rad = math.radians((max_angle/2)*steering)
        
        result = (rad-self.steer)/self.stamped_time
        self.steer = rad
        return result

def main():
    rclpy.init()
    velocity_receiver = VelocityReceiver()

    rclpy.spin(velocity_receiver)
    velocity_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
