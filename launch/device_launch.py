import rclpy
import os
import signal
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import launch.actions


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


    gui_node = Node(
        package="naviscope",
        namespace=f"user_{INDEX}",
        executable="gui",
        name='gui',
        output='screen'

    )
  

    # Add the nodes and the process to the LaunchDescription list
    ld = LaunchDescription()
    ld.add_action(heartbeat_node)
    ld.add_action(gui_node)
    #ld.add_action(unityTCP_node)
    
    # Register the shutdown handler
    shutdown_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=gui_node,  # Specify the node whose exit will trigger the shutdown
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    ld.add_action(shutdown_event_handler)

    return ld


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    ld = generate_launch_description()
    print("Launching nodes...")
    rclpy.init()
    rclpy.spin(ld)
    rclpy.shutdown()