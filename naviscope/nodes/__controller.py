#!/usr/bin/env python3
########################################################################
# Filename    : __controller.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import sys
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int8, UInt16

from ..utils.__utils_objects import AVAILABLE_TOPICS, OPERATOR

from ..components.__pushHoldBtn import pushHoldButton
from ..components.__rotaryEncoder import rotaryEncoder


class OperatorNode( Node ):

        def __init__( self, **kwargs):

            super().__init__("controller", namespace="operator_0")
            
            self._sub_sensors = None

            self._pinout_steering = {
                    "clk" : 6,
                    "dt" : 4
            }

            self._pinout_propulsion = {
                    "clk" : 6,
                    "dt" : 4,
                    "sw" : 0
            }

            self._pub_propulsion = None
            self._pub_direction = None
            self._pub_orientation = None

            self._timer = None 

            self._watchdog_sub = None

            self._comp_propulsion = None    
            self._comp_orientation = None
            self._comp_audio = None

            self._is_peer_connected = False
            self._force_commands_enabled = False

            self._operator_type = OPERATOR.USER.value

            self._peer_event_triggered = False

            self.start()


        def start(self):
            
            self._declare_parameters()

            self._init_subscribers()
            self._init_publishers()

            self._init_component()

        def _declare_parameters( self ):

            self.declare_parameter("verbose", False)
            self.declare_parameter("peer_index", 0)


        def _init_component(self):
            
            self._comp_direction = pushHoldButton( 
                pin = 6,
                callback = self._update_direction,
                hold_threshold=3
            )

            self._comp_propulsion = rotaryEncoder(
                pin_clk = 6,
                pin_dt = 5,
                enableRange = True,
                incrementFactor=5,
                minClip = 50,
                maxClip = 200,
                callback = self._update_propulsion
            )

            self._comp_orientation = rotaryEncoder(
                pin_clk = 6,
                pin_dt = 5,
                incrementFactor=10,
                callback = self._update_orientation
            )


        def _init_subscribers( self ):
            
            self._sub_sensors = self.create_subscription(
                String,
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.SENSOR.value}",
                self._manage_sensors_datas,
                10
            )

            self._sub_sensors 

            self._watchdog_sub = self.create_subscription(
                Bool,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self._react_to_connections,
                10
            )

            self._watchdog_sub   


        def _init_publishers( self ):


            self._pub_propulsion  = self.create_publisher(
                UInt16, 
                AVAILABLE_TOPICS.PROPULSION.value,
                10
            )
            
            self._pub_propulsion

            self._pub_direction = self.create_publisher(
                Int8, 
                AVAILABLE_TOPICS.DIRECTION.value,
                10
            )
            
            self._pub_direction

            self._pub_orientation  = self.create_publisher(
                Int8, 
                AVAILABLE_TOPICS.ORIENTATION.value,
                10
            )
            
            self._pub_orientation


        def _update_propulsion( self, update_pwm_range = 100 ):
            
            msg = UInt16()
            msg.data = int(update_pwm_range) 

            self._pub_propulsion.publish( msg )


        def _update_direction( self, spin_direction = 0 ):
            
            msg = Int8()
            msg.data = spin_direction

            self._pub_direction.publish( msg )

            if spin_direction == 0:
                self._comp_propulsion.reset()

        def _update_orientation( self, increment = 0 ):

            msg = Int8()
            msg.data = int(increment)

            self._pub_orientation.publish( msg )


        def _manage_sensors_datas( self, msg ): 

            datas = json.loads( msg.data )


        def _react_to_connections( self, msg ):

            self._is_peer_connected = msg.data

            """
            if self._is_peer_connected is False:

                if self._component is not None:
                    
                    print( "peerIsConnected (drone)", self._is_peer_connected )
            """


        def exit(self):

            self.get_logger().info("shutdown heartbeat")   
            self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    controller_node = None 

    try:

        controller_node = OperatorNode()

        rclpy.spin(controller_node )

    except Exception as e:
        print( "an exception has been raised while spinning controller node : ", e )
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:

        if controller_node is not None:
            controller_node.exit()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()