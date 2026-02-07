from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult

class SliderNode(Node):

    def __init__(self):
        super().__init__('slider_parameter_node')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'
        joint_state = JointState()


        param_range = FloatingPointRange(from_value=-360.0, to_value=360.0, step=0.0)
        param_descriptor = ParameterDescriptor(floating_point_range=[param_range])

        self.declare_parameter('yaw', 0.0, param_descriptor)
        self.declare_parameter('pitch', 0.0, param_descriptor)
        self.declare_parameter('roll', 0.0, param_descriptor)
        self.declare_parameter('use_quaternions', False)
        self.use_quaternions = False
        self.add_on_set_parameters_callback(self.slider_callback)

        self.get_logger().info("Node Started, open rqt_reconfigure")



    def slider_callback(self, params):
        for param in params:

            if param.name == 'use_quaternions':
                self.get_logger().info(f"Use Quaternions?: {param.value}")
                self.use_quaternions = param.value

            if param.name == 'roll':
                self.get_logger().info(f"Roll: {param.value}")

            if param.name == 'pitch':
                self.get_logger().info(f"Pitch: {param.value}")

            if param.name == 'yaw':
                self.get_logger().info(f"Yaw: {param.value}")

        return SetParametersResult(successful=True)



def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = SliderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
