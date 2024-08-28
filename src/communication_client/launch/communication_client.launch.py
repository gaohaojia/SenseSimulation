from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    network_port = LaunchConfiguration('network_port')
    network_ip = LaunchConfiguration('network_ip')
    
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    declare_network_port = DeclareLaunchArgument('network_port', default_value='12131', description='')
    declare_network_ip = DeclareLaunchArgument('network_ip', default_value='192.168.31.207', description='')

    communication_server_node = Node(
        package='communication_client',
        executable='communication_client_node',
        name='communication_client',
        output='screen',
        respawn=True,
        parameters=[{
            'robot_id': robot_id,
            'network_port': network_port,
            'network_ip': network_ip
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_id)
    ld.add_action(declare_network_port)
    ld.add_action(declare_network_ip)
    
    ld.add_action(communication_server_node)
    
    return ld