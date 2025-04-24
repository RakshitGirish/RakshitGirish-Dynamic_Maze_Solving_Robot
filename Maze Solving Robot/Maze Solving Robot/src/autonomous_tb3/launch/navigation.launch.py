from launch import LaunchDescription
from launch_ros.actions import Node  
import os  
 
from ament_index_python.packages import get_package_share_directory 
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import SetEnvironmentVariable
config_dir_share_path = os.path.join(get_package_share_directory('autonomous_tb3'), 'config') 
map_config_file_path = os.path.join(config_dir_share_path, "tb3_world_2d_map.yaml")    
params_config_file_path = os.path.join(config_dir_share_path, "tb3_nav_params.yaml") 
rviz_config_file_path = os.path.join(config_dir_share_path, "tb3_nav.rviz")  
turtlebot3_gazebo_share_path = get_package_share_directory('turtlebot3_gazebo')
nav2_bringup_share_path = get_package_share_directory('nav2_bringup')
           

def generate_launch_description():
    
    return LaunchDescription([
        
        SetEnvironmentVariable(
            name = "TURTLEBOT3_MODEL",
            value="waffle"
        ),
        
 
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource (
                 launch_file_path=Path(turtlebot3_gazebo_share_path, "launch/turtlebot3_world.launch.py").as_posix()
             )
         ),
        
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource (
                 launch_file_path=Path(nav2_bringup_share_path, "launch/bringup_launch.py").as_posix()
             ),
             launch_arguments = {
                 'map' : map_config_file_path,
                 'params_file' : params_config_file_path
                 }.items(),
         ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_node",   
            arguments = ['-d', rviz_config_file_path],
            output="screen"
        )                     
    ])