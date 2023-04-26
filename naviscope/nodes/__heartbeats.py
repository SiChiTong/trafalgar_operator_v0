#!/usr/bin/env python3
########################################################################
# Filename    : __heartbeats.py
# Description : check if an operator is connected and send emergency cmd to other nodes
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import sys
import json
import socket  
from time import process_time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from ..utils.__utils_objects import AVAILABLE_TOPICS, OPERATOR

class HeartbeatsNode( Node ):

        def __init__( self, **kwargs ):

            super().__init__("heartbeat", namespace="operator_0")
            
            self._address  = None 
            self._heartbeats = None
            self._beat_pulsation = 1.0

            self._watchdog_pub = None
            self._pub_shutdown = None

            self._peer_sub = None
            self._peer_timer = None
            self._peer_timeout = 2.0
            self._peer_pulse_time = 1.0
            self._is_peer_connected = False

            self._is_master_connected = False

            self._operator_type = OPERATOR.USER.value

            self.start()


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
            
            self._peer_sub = self.create_subscription(
                String, 
                f"/drone_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self._on_peer_pulse,
                10
            )

            self._peer_sub 
            self._peer_timer = self.create_timer( self._peer_timeout, self._check_peer_status )


        def _init_publishers( self ):

            self._heartbeats = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.HEARTBEAT.value,
                10
            )

            self.timer = self.create_timer( self._beat_pulsation, self._pulse)

            #watchdog publisher => send instruction when the operator connection status change
            self._watchdog_pub = self.create_publisher(
                Bool, 
                AVAILABLE_TOPICS.WATCHDOG.value,
                10
            )

            self._pub_shutdown = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.SHUTDOWN.value,
                10
            )

            self._heartbeats
            self._watchdog_pub
            self._pub_shutdown


        def _pulse( self ):
            
            if self._heartbeats is not None: 
                
                pulse_msg = String()

                info = {
                    "address" : self._address,
                    "operator" : self._operator_type,
                    "pulse" : process_time()
                }

                pulse_msg.data = json.dumps( info )

                self._heartbeats.publish( pulse_msg )


        def _on_peer_pulse( self, pulse_msg ):

            if not self._is_peer_connected :

                peer_status_msg = Bool()
                peer_status_msg.data = True

                self._watchdog_pub.publish(peer_status_msg)
                
            self._is_peer_connected = True


        def _check_peer_status(self):

            if not self._is_peer_connected :
                        
                peer_status_msg = Bool()
                peer_status_msg.data = False
                
                self._watchdog_pub.publish( peer_status_msg )

                #print( "operator is disconnected to the drone")
                    
            self._is_peer_connected =  False


        def exit( self ):
            self.get_logger().info("shutdown heartbeat")
            self.destroy_node()  
  

def main(args=None):

    rclpy.init(args=args)

    heartbeats_node_pub = None 

    try:

        heartbeats_node_pub =  HeartbeatsNode()

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