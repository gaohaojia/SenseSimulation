from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    real_to_simulation_node = Node(
        package='real_to_simulation',
        executable='real_to_simulation_node',
        name='real_to_simulation',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(real_to_simulation_node)
    
    return ld