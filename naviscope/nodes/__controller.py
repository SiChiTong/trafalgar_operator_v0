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

            super().__init__("controller", namespace=f"{PEER.USER.value}_0")
          
            self._pub_propulsion = None
            self._pub_direction = None
            self._pub_orientation = None
            self._pub_pantilt = None

            self._board = None
            
            self._propulsion_default = 25 #default percentage of thrust
            self._propulsion = self._propulsion_default 
            self._direction = 0
            self._orientation = 0
  
            self._delta_yaw = 0
            self._delta_roll = 0
            self._delta_pitch = 0

            self._sensor_yaw = 0
            self._sensor_pitch =0
            self._sensor_roll =0
            
            self._angleX = 90
            self._angleZ = 90

            self.start()


        def start(self):


            self._declare_parameters()
            self._init_publishers()

            self._init_component()


        def reset( self ):

            self._propulsion = self._propulsion_default 
            self._direction = 0
            self._orientation = 0
            self._angleX = 90
            self._angleZ = 90

        def _declare_parameters( self ):
            self.declare_parameter("peer_index", 0)

        def _init_component(self):
            
            self._board = externalBoard( self._board_datas )
            self.reset()

            self._board._enable()


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
                qos_profile=qos_profile_sensor_data
            )

            self._pub_pantilt


        def _update_propulsion( self ):
            
            prop_msg = UInt16()
            prop_msg.data = self._propulsion

            self._pub_propulsion.publish( prop_msg )


        def _update_direction( self ):
            
            spin = int(np.clip( self._direction, -1, 1 ))
          
            if self._direction != spin: 

                if( spin == 0):
                    self.reset()

                self._direction = spin

                dir_msg = Int8()
                dir_msg.data = spin

                self._pub_direction.publish( dir_msg )

        def _update_orientation( self, increment = 0 ):
            
            if increment != 0:
                msg = Int8()
                msg.data = int(increment)

                self._pub_orientation.publish( msg )

        def _update_panoramic( self, pan = 90, tilt=90):

            update_tilt = tilt
            update_pan = pan

            vec = Vector3()
            vec.x = float( update_tilt )
            vec.z = float( update_pan )

            self._pub_pantilt.publish( vec )


        def _board_datas( self, json_datas ): 
            
            datas = json_datas

            self.OnNewOrientation(datas["orientation"])
            self.OnNewPropulsion(datas["propulsion"])
  
            self.OnButtonPress( datas["shortPress"], datas["longPress"] )

            self.OnMPUDatas(
                pitch=datas["pitch"],
                roll=datas["roll"],
                yaw=datas[ "yaw" ],
                delta_p = datas[ "delta_pitch" ],
                delta_r=datas[ "delta_roll" ],
                delta_y=datas[ "delta_yaw" ]
            )

        def OnNewOrientation( self, increment ):
            self._update_orientation( increment )

        def OnNewPropulsion( self, updateLevelIncrement ): 

            if self._direction != 0:

                increment = self._propulsion + updateLevelIncrement
                
                if self._direction > 0:
                    increment = math.floor( np.clip( increment, 25, 100 ) ) 
                elif self._direction < 0 :
                    increment = math.floor( np.clip( increment, 25, 50 ) ) 

                if increment != self._propulsion:

                    self._propulsion = increment
                    self._update_propulsion()


        def OnButtonPress( self, shortPress = False, longPress = False ):
            
            if longPress is True:

                if self._direction != -1:
                    self._direction = -1
                    self._update_direction()

            if shortPress is True:

                if self._direction == 0:
                    self._direction = 1
                else:
                    self._direction = 0

                self._update_direction()

        def OnMPUDatas( self, pitch=0, roll=0, yaw=0, delta_p = 0, delta_r=0, delta_y=0 ):

            self._sensor_yaw = yaw
            self._sensor_pitch = pitch
            self._sensor_roll = roll

            self._delta_p = delta_p
            self._delta_r =  delta_r
            self._delta_y = delta_y

            angle_aroundX = np.clip(90 + delta_r, 0, 180)
            angle_aroundY = np.clip( 90 - delta_y, 0,180 )
            angle_aroundZ = np.clip( 90 + delta_p, 0,180 )

            self._angleX = angle_aroundY
            self._angleZ = np.clip( self._angleZ + delta_p, 0,180 )

            self._update_panoramic(pan=self._angleZ, tilt = self._angleX  )


        def exit(self):

            print("shutdown controller")   



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

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()