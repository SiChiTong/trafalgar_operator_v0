import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Declare a variable Node for each node
    INDEX = int(os.environ.get('PEER_ID'))

    heartbeat_node = Node(
        package="naviscope",
        namespace=f"user_{INDEX}",
        executable="heartbeat",
        name='heartbeat',
        parameters=[{
            "peer_index":INDEX
        }]


    )

    controller_node = Node(
        package="naviscope",
        namespace=f"user_{INDEX}",
        executable="controller",
        name='controller',
        parameters=[{
            "peer_index":INDEX
        }]

    )


    gui_node = Node(
        package="naviscope",
        namespace=f"user_{INDEX}",
        executable="gui",
        name='gui'

    )
  

    # Add the nodes and the process to the LaunchDescription list
    ld = [
     
        heartbeat_node,
        controller_node,
        gui_node
    ]

    return LaunchDescription(ld)