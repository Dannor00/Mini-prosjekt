from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(*args):
    # 🔧 PID-kontroller kommando
    pid_cmd_str = (
        f'source ~/qube_ws/install/setup.bash && '
        f'ros2 run qube_controller pid_node '
    )
    pid_cmd = ['gnome-terminal', '--', 'bash', '-c', pid_cmd_str]
    
    return [
        ExecuteProcess(cmd=pid_cmd, shell=False),
    ]
    

def generate_launch_description():
    
    # Paths
    pkg_bringup = get_package_share_directory('qube_bringup')
    urdf_path = os.path.join(pkg_bringup, 'urdf', 'controlled_qube.urdf.xacro')
    pkg_driver_path= os.path.join(
            get_package_share_directory('qube_driver'), 
            'launch', 
            'qube_driver.launch.py'
    )

    robot_description = Command(['xacro ', urdf_path])
   
    return LaunchDescription([
        
        OpaqueFunction(function=launch_setup),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pkg_driver_path)
        ),
       
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[],
            output='screen',
            condition=None
        ),
        
    ])
