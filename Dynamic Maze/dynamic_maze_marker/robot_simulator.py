import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        self.publisher = self.create_publisher(Marker, 'robot_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_robot_marker)

        # Start position (can be dynamic later)
        self.x = 1
        self.y = 1
        self.scale = 1.0

    def publish_robot_marker(self):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x * self.scale
        marker.pose.position.y = self.y * self.scale
        marker.pose.position.z = 0.5 * self.scale
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.scale * 0.7
        marker.scale.y = self.scale * 0.7
        marker.scale.z = self.scale * 0.7
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.lifetime.sec = 1

        self.publisher.publish(marker)
        self.get_logger().info('Published robot marker at (%d, %d)' % (self.x, self.y))

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

