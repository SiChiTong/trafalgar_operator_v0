from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Declare a variable Node for each node
    INDEX = 0
    VERBOSE = False
    OPENCV_RENDER = False

    heartbeat_node = Node(
        package="naviscope",
        namespace=f"operator_{INDEX}",
        executable="heartbeat",
        name='heartbeat',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index":INDEX
        }]


    )

    controller_node = Node(
        package="naviscope",
        namespace=f"operator_{INDEX}",
        executable="controller",
        name='controller',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index":INDEX
        }]

    )

    videostream_node = Node(
        package="naviscope",
        namespace=f"operator_{INDEX}",
        executable="videostream",
        name='videostream',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index":INDEX,
            "resolution" : (320,240),
            "opencv_render": OPENCV_RENDER
        }]

    )

    imu_node = Node(
        package="naviscope",
        namespace=f"operator_{INDEX}",
        executable="imu",
        name='imu',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index":INDEX
        }]

    )


    # Add the nodes and the process to the LaunchDescription list
    ld = [
     
        heartbeat_node,
        controller_node,
        imu_node,
        videostream_node
    ]

    return LaunchDescription(ld)