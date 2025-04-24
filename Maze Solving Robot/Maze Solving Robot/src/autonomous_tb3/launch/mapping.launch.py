from launch import LaunchDescription
from launch_ros.actions import Node   
from ament_index_python.packages import get_package_share_directory 
import os    

config_dir = os.path.join(get_package_share_directory('autonomous_tb3'), 'config')                            

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package="cartographer_ros",                       
            executable="cartographer_node",                  
            name="cart_node",   
            arguments=["-configuration_directory", config_dir, "-configuration_basename", "turtlebot3_cartographer.lua"],                         
            output="screen"
        ),
        
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cart_og_node", 
            output="screen"
        ),
                        
    ])