import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import random

class MazeMarkerPublisher(Node):
    def __init__(self):
        super().__init__('maze_marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'maze_markers', 10)
        self.timer = self.create_timer(2.0, self.publish_maze)
        self.width = 10
        self.height = 10
        self.scale = 1.0  # cube size in meters

    def publish_maze(self):
        marker_array = MarkerArray()
        marker_id = 0

        entry_point = (0, 1)  # You can change this as needed
        exit_point = (self.width - 1, self.height - 2)  # Change this too

        for y in range(self.height):
            for x in range(self.width):
                is_boundary = x == 0 or y == 0 or x == self.width - 1 or y == self.height - 1
                is_entry = (x, y) == entry_point
                is_exit = (x, y) == exit_point

                if is_boundary or random.random() < 0.3:  # outer wall or random inner wall
                    marker = Marker()
                    marker.header = Header()
                    marker.header.frame_id = "map"
                    marker.ns = "maze"
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x * self.scale
                    marker.pose.position.y = y * self.scale
                    marker.pose.position.z = 0.5 * self.scale
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = self.scale
                    marker.scale.y = self.scale
                    marker.scale.z = self.scale

                    # Color markers based on their type
                    if is_entry:
                        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green for entry
                    elif is_exit:
                        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red for exit
                    elif is_boundary:
                        marker.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=1.0)  # Gray for boundaries
                    else:
                        marker.color = ColorRGBA(r=0.1, g=0.1, b=1.0, a=1.0)  # Blue for inner walls

                    marker.lifetime.sec = 2  # auto-remove
                    marker_array.markers.append(marker)
                    marker_id += 1

        self.publisher.publish(marker_array)
        self.get_logger().info('Published maze with entry and exit points.')

def main(args=None):
    rclpy.init(args=args)
    node = MazeMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

