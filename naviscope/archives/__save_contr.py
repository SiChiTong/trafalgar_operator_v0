#!/usr/bin/env python3
########################################################################
# Filename    : __controller.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

import json
import numpy as np

import odroid_wiringpi as wiringpi

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int8, Uint8, Uint16
from sensor_msgs.msg import Imu

from ..utils.__utils_objects import AVAILABLE_TOPICS, OPERATOR

from ..components.__btn_propulsion import propulsionButton
from ..components.__rot_orientation import orientationEncoder
from .__imu_view import viewMPU

class Operator( Node ):

        def __init__( self, **kwargs):

            super().__init__("controller", namespace="operator_0")
            
            self._sub_sensors = None

            self._pub_propulsion = None
            self._pub_direction = None
            self._pub_orientation = None
            self._pub_tilt = None
            self._pub_pan = None

            self._timer = None 

            self._watchdog_sub = None

            self._comp_propulsion = None    
            self._comp_orientation = None
            self._comp_imu = None
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

        def _init_board( self ):
            wiringpi.wiringPiSetup()

        def _init_component(self):
            
            self._init_board()

            self._comp_propulsion = propulsionButton( 
                self,
                pin = 6,
                backward_threshold = 3
            )

            self._comp_orientation = orientationEncoder(
                master = self,
                pin_clk = 4,
                pin_dt = 5
            )

            self._comp_imu = viewMPU(
                master = self,
                address = 0x68
            )

            self._timer = self.create_timer( 0.1, self.read_imu_data )


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
                String, 
                AVAILABLE_TOPICS.PROPULSION.value,
                10
            )
            
            self._pub_propulsion

            self._pub_direction = self.create_publisher(
                Uint16, 
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

            self._pub_pan = self.create_publisher(
                Uint8, 
                AVAILABLE_TOPICS.PAN.value,
                10
            )

            self._pub_pan

            self._pub_tilt = self.create_publisher(
                Uint8, 
                AVAILABLE_TOPICS.TILT.value,
                10
            )

            self._pub_tilt

        def read_imu_data( self ):

            if self._comp_imu is not None: 
                self._comp_imu.read()
                
                
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = math.sin(orientation / 2.0)
                imu_msg.orientation.w = math.cos(orientation / 2.0)
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az

        self.imu_pub.publish(imu_msg)

        def _update_propulsion( self, update_pwm = 100 ):
            
            msg = Uint16()
            msg.data = int(update_pwm) 

            self._pub_propulsion.publish( msg )


        def _update_direction( self, spin_direction = "stp" ):
            
            msg = String()
            msg.data = spin_direction

            self._pub_direction.publish( msg )


        def _update_orientation( self, increment = 0 ):

            msg = Int8()
            msg.data = int(increment)

            self._pub_orientation.publish( msg )


        def _update_pan( self, angle = 90 ):

            msg = Uint8()
            msg.data = int(angle)

            self._pub_pan.publish( msg )


        def _update_tilt( self, angle = 90 ):

            msg = Uint8()
            msg.data = int(angle)

            self._pub_tilt.publish( msg )


        def _manage_sensors_datas( self, msg ): 

            datas = json.loads( msg.data )


        def _react_to_connections( self, msg ):

            self._is_peer_connected = msg.data
            #print( "operator status message received, connection ", msg.peer_connected)
            
            """
            if self._is_peer_connected is False:

                if self._component is not None:
                    
                    print( "peerIsConnected (drone)", self._is_peer_connected )
            """


        def exit(self):
            
            self.destroy_node()
            print("shutdown")



def main(args=None):

    rclpy.init(args=args)

    controller_node = None 

    try:

        controller_node = Operator()

        rclpy.spin(controller_node )

    except Exception as exception:
        print( "an exception has been raised while spinning heartbeats : ", exception)

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:

        if controller_node is not None:
            controller_node.exit()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()