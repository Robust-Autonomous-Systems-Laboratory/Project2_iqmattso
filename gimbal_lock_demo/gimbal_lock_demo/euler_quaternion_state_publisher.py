import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult

class SliderNode(Node):

    def __init__(self):
        super().__init__('slider_parameter_node')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()

        param_range = FloatingPointRange(from_value=-360.0, to_value=360.0, step=0.0)
        param_descriptor = ParameterDescriptor(floating_point_range=[param_range])

        self.declare_parameter('yaw', 0.0, param_descriptor)
        self.declare_parameter('pitch', 0.0, param_descriptor)
        self.declare_parameter('roll', 0.0, param_descriptor)
        self.declare_parameter('use_quaternions', False)
        self.add_on_set_parameters_callback(self.slider_callback)

        self.use_quaternions = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.get_logger().info("Node Started, open rqt_reconfigure")


    def slider_callback(self, params):

        # first, update params
        for param in params:

            if param.name == 'use_quaternions':
                self.get_logger().info(f"Use Quaternions?: {param.value}")
                self.use_quaternions = param.value

            if param.name == 'roll':
                self.get_logger().info(f"Roll: {param.value}")
                self.roll = param.value

            if param.name == 'pitch':
                self.get_logger().info(f"Pitch: {param.value}")
                self.pitch = param.value

            if param.name == 'yaw':
                self.get_logger().info(f"Yaw: {param.value}")
                self.yaw = param.value

        # then, populate and publish a JointState() msg
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['roll_joint', 'pitch_joint', 'yaw_joint']
        

        if self.use_quaternions:
            # directly control euler angles
            msg.position = [self.roll, self.pitch, self.yaw]

        else:
            # convert from euler to quaternions
            quaternion_rotation = R.from_euler('zyx', [self.roll, self.pitch, self.yaw])
            r, p, y = quaternion_rotation.as_euler('xyz')
            msg.position = [r, p, y]

        self.joint_pub.publish(msg)

        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = SliderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
