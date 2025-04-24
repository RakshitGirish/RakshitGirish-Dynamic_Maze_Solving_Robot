#! /usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np


class Occupancy_Grid_Publisher(Node):

    def __init__(self):
        super().__init__('occupancy_grid_pub_node')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'occupancy_grid_map', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        msg = OccupancyGrid()   

        msg.header = Header() 

        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = 'map_frame' 
             
        msg.info.resolution = 1.0
        msg.info.width = 3      
        msg.info.height = 3    
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        
        
        msg.data = np.array([0,1,1,0,1,1,1,1,0], dtype=np.int8).tolist()
        #  ______ ______ ______
        # |      |      |      |
        # |   0  |  0   |  1   |
        # |______|______|______|
        # |      |      |      |
        # |   1  |  1   |  1   |
        # |______|______|______|
        # |      |      |      |
        # |   1  |  1   |  0   |
        # |______|______|______|
        
        
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_publisher = Occupancy_Grid_Publisher()
    print('Publishing Map...')
    rclpy.spin(occupancy_grid_publisher)
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
