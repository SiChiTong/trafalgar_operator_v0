#!/usr/bin/env python3
########################################################################
# Filename    : __heartbeats.py
# Description : check if an operator is connected and send emergency cmd to other nodes
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
import sys
import json
import socket  
from time import process_time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_sensor_data

from ..utils.__utils_objects import AVAILABLE_TOPICS, PEER, EXIT_STATE

class HeartbeatsNode( Node ):

        def __init__( self, **kwargs ):

            super().__init__("heartbeat", namespace=f"{PEER.USER.value}_0")
            
            self._address  = None 
            self._heartbeats = None
            self._beat_pulsation = 1.0

            self._pub_watchdog = None
            self._sub_shutdown = None
            self._sub_peer = None

            self._peer_timer = None
            self._peer_timeout = 2.0
            self._peer_pulse_time = 1.0
            self._is_peer_connected = False

            self._is_master_connected = False

            self._peer_type = PEER.USER.value

            self.start()


        def _kill_instruction():
            os.system("shutdown /s /t 1")

        def _restart_instruction():
            os.system("shutdown /r /t 1")
            

        def _react_to_shutdown_cmd(self, msg ): 

            instruction = json.loads(msg.data)
                
            operator = instruction["peer"]
            instruction = instruction["status"]

            if( instruction == EXIT_STATE.RESTART.value ):

                self._restart_instruction()

            elif ( instruction == EXIT_STATE.SHUTDOWN.value ):

                self._kill_instruction()


        def start( self ):

            self.get_local_ip()
            self._declare_parameters()

            self._init_publishers()
            self._init_subscribers()


        def get_local_ip( self ):

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            try:

                s.connect(('8.8.8.8', 80))
                self._address = s.getsockname()[0]
            except socket.error:
                # Si la connexion échoue, nous renvoyons l'adresse IP de la machine locale
                self._address = socket.gethostbyname(socket.gethostname())
            finally:
                s.close()
            

        def _declare_parameters( self ):

            self.declare_parameter("verbose", False)
            self.declare_parameter("peer_index", 0)

        def _init_subscribers( self ):
            
            self._sub_shutdown = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.SHUTDOWN.value}",#operator_{self.get_parameter('peer_index').value}_
                self._react_to_shutdown_cmd,
                10
            )

            self._sub_shutdown
        
            self._sub_peer = self.create_subscription(
                String, 
                f"/{PEER.DRONE.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._on_peer_pulse,
                qos_profile=qos_profile_sensor_data
            )

            #listen for master deconnection
            
            self._sub_peer 
            self._peer_timer = self.create_timer( self._peer_timeout, self._check_peer_status )


        def _init_publishers( self ):

            self._heartbeats = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.HEARTBEAT.value,
                qos_profile=qos_profile_sensor_data
            )

            self.timer = self.create_timer( self._beat_pulsation, self._pulse)

            #watchdog publisher => send instruction when the operator connection status change
            self._pub_watchdog = self.create_publisher(
                Bool, 
                AVAILABLE_TOPICS.WATCHDOG.value,
                10
            )

            self._heartbeats
            self._pub_watchdog


        def _pulse( self ):
            
            if self._heartbeats is not None: 
                
                pulse_msg = String()

                info = {
                    "address" : self._address,
                    "peer" : self._peer_type,
                    "pulse" : process_time()
                }

                pulse_msg.data = json.dumps( info )

                self._heartbeats.publish( pulse_msg )


        def _on_peer_pulse( self, pulse_msg ):

            if not self._is_peer_connected :

                peer_status_msg = Bool()
                peer_status_msg.data = True

                #self._operator_connect_event.set()

                self._pub_watchdog.publish(peer_status_msg)

                #print( "operator is connected to the drone")

            self._is_peer_connected = True

            #print( "operator pulse time", pulse_msg.pulse_time)


        def _check_peer_status(self):

            if not self._is_peer_connected :

                peer_status_msg = Bool()
                peer_status_msg.data = False
                
                self._pub_watchdog.publish( peer_status_msg )

                print( "operator is disconnected to the drone")
                    
            #self._timer_handshake_done = False
            self._is_peer_connected =  False


        def exit( self ):
            self.destroy_node()  
            print("shutdown heartbeat")



def main(args=None):

    rclpy.init(args=args)

    heartbeats_node_pub = None 

    try:

        heartbeats_node_pub = HeartbeatsNode()

        rclpy.spin(heartbeats_node_pub )

    except Exception as e:
        print( "an exception has been raised while spinning heartbeats node : ", e )
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:

        if heartbeats_node_pub is not None:
            heartbeats_node_pub.exit()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()