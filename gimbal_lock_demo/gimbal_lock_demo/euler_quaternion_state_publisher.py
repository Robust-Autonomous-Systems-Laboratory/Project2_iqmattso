import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult

class SliderNode(Node):

    def __init__(self):
        super().__init__('slider_parameter_node')

        param_range = FloatingPointRange(from_value=-360.0, to_value=360.0, step=0.0)
        param_descriptor = ParameterDescriptor(floating_point_range=[param_range])

        self.declare_parameter('euler_yaw', 0.0, param_descriptor)
        self.add_on_set_parameters_callback(self.slider_callback)

        self.get_logger().info("Node Started, open rqt_reconfigure")


    def slider_callback(self, params):
        for param in params:
            if param.name == 'euler_yaw':
                self.get_logger().info(f"Euler Yaw: {param.value}")

        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = SliderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
