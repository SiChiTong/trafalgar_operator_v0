#!/usr/bin/env python3
########################################################################
# Filename    : __controller.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import sys
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int8, UInt16
from geometry_msgs.msg import Vector3

from rclpy.qos import qos_profile_sensor_data

from ..components.__microcontroller import externalBoard
from ..utils.__utils_objects import AVAILABLE_TOPICS, PEER

class OperatorNode( Node ):

        def __init__( self, **kwargs):

            super().__init__("controller", namespace="operator_0")
          
            self._pub_propulsion = None
            self._pub_direction = None
            self._pub_orientation = None
            self._pub_pantilt = None

            self._watchdog_sub = None

            self._board = None
            self._is_peer_connected = False
   
            self._operator_type = PEER.USER.value

            self._propulsion = 0
            self._direction = 0
            self._orientation = 0
  
            self._delta_yaw = 0
            self._delta_roll = 0
            self._delta_pitch = 0

            self._sensor_yaw = 0
            self._sensor_pitch =0
            self._sensor_roll =0
            
            self._pitch = 90
            self._yaw = 90
            self._roll = 90

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
            
            self._board = externalBoard( self._board_datas )
            self._board._enable()

        def _init_subscribers( self ):
            
            self._sub_sensors = self.create_subscription(
                String,
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.SENSOR.value}",
                self._drone_sensors_feedback,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_sensors 

            self._watchdog_sub = self.create_subscription(
                Bool,
                AVAILABLE_TOPICS.WATCHDOG.value,
                self._react_to_connections,
                qos_profile=qos_profile_sensor_data
            )

            self._watchdog_sub   


        def _init_publishers( self ):

            self._pub_propulsion  = self.create_publisher(
                UInt16, 
                AVAILABLE_TOPICS.PROPULSION.value,
                qos_profile=qos_profile_sensor_data
            )
            
            self._pub_propulsion

            self._pub_direction = self.create_publisher(
                Int8, 
                AVAILABLE_TOPICS.DIRECTION.value,
                qos_profile=qos_profile_sensor_data
            )
            
            self._pub_direction

            self._pub_orientation  = self.create_publisher(
                Int8, 
                AVAILABLE_TOPICS.ORIENTATION.value,
                qos_profile=qos_profile_sensor_data
            )
            
            self._pub_orientation

            self._pub_pantilt = self.create_publisher(
                Vector3,
                AVAILABLE_TOPICS.PANTILT.value,
                10
            )

            self._pub_pantilt


        def _update_propulsion( self, update_pwm = 100 ):
            
            increment = math.floor( np.clip( update_pwm, 50, 200 ) ) 
            self._propulsion = increment

            prop_msg = UInt16()
            prop_msg.data = increment

            self._pub_propulsion.publish( prop_msg )


        def _update_direction( self, spin_direction = 0 ):
            
            spin = int(np.clip( spin_direction, -1, 1 ))

            if self._direction != spin: 

                if( spin == 0):
                    self._yaw = 90
                    self._pitch = 90

                self._direction = spin

                dir_msg = Int8()
                dir_msg.data = spin

                self._pub_direction.publish( dir_msg )


        def _update_orientation( self, increment = 0 ):

            msg = Int8()
            msg.data = int(increment)

            self._pub_orientation.publish( msg )


        def _update_panoramic( self ):

            #tilt = np.clip( self._pitch, 0, 180 )
            #azimuth = np.clip( self._yaw, 0, 180 )
            tilt = self._pitch
            azimuth = self._yaw
            #tilt -90 to 90
            vec = Vector3()
            vec.x = float( tilt )
            vec.z = float( azimuth )

            self._pub_pantilt.publish( vec )


        def _board_datas( self, datas ): 
            
            #print(f" propulsion : {datas[ 'propulsion' ]}, propulsion : {datas[ 'orientation' ]}, direction : {datas[ 'direction' ]}")
        
            if datas[ "propulsion" ] != self._propulsion:
                self._update_propulsion( datas[ "propulsion" ] )

            if datas[ "orientation" ] != self._orientation:
                self._update_orientation( datas["orientation"] )

            if datas[ "direction" ] != self._direction:
                self._update_direction( datas["direction"] )
                        
            self._sensor_yaw = datas[ "yaw" ]
            self._sensor_pitch = datas["pitch"]
            self._sensor_roll = datas["roll"]

            self._delta_yaw = datas[ "delta_yaw" ]
            self._delta_pitch = datas["delta_pitch"]

            self._yaw = 90 - self._sensor_yaw
            self._pitch += self._delta_pitch

            self._update_panoramic( )
            

        def _react_to_connections( self, msg ):

            self._is_peer_connected = msg.data

            """
            if self._is_peer_connected is False:

                if self._component is not None:
                    
                    print( "peerIsConnected (drone)", self._is_peer_connected )
            """


        def _drone_sensors_feedback( self, msg ): 
            
            feedback = json.loads( msg.data )


        def exit(self):

            self.get_logger().info("shutdown controller")   
            self.destroy_node()



def main(args=None):

    rclpy.init(args=args)

    controller_node = None 

    try:

        controller_node = OperatorNode()

        rclpy.spin( controller_node )

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

        rclpy.shutdown()


if __name__ == '__main__':
    main()