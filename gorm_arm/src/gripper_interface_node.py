import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class GripperInterfaceNode(Node):

    def __init__(self):
        super().__init__('gripper_interface_node')
        self.subscription = self.create_subscription(
            Int32,
            'topic',
            self.gripper_pos_callback,
            10)
        self.subscription  # prevent unused variable warning

    def gripper_pos_callback(self, msg):
        self.get_logger().info('I heard:',msg.data)


def main(args=None):
    rclpy.init(args=args)

    gripper_interface_node = GripperInterfaceNode()

    rclpy.spin(gripper_interface_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()