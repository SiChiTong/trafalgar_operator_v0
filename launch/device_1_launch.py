from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Declare a variable Node for each node
    INDEX = 1
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

    videostream_node = Node(
        package="naviscope",
        namespace=f"user_{INDEX}",
        executable="videostream",
        name='videostream',
        parameters=[{
            "peer_index":INDEX,
            "resolution" : (320,180)
        }]

    )


    # Add the nodes and the process to the LaunchDescription list
    ld = [
     
        heartbeat_node,
        controller_node,
        videostream_node
    ]

    return LaunchDescription(ld)