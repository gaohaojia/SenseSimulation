from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    communication_client_node = Node(
        package="communication_server",
        executable="communication_server_node",
        name="communication_server",
        output="screen",
        respawn=True
    )

    ld = LaunchDescription()
    ld.add_action(communication_client_node)

    return ld
