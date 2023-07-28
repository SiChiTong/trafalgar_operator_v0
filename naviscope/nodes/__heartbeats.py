#!/usr/bin/env python3
########################################################################
# Filename    : __heartbeats.py
# Description : check if an operator is connected and send emergency cmd to other nodes
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import sys
import subprocess
import json
import socket  
#from time import process_time

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
            self._sub_master_pulse = None

            self._peer_timer = None
            self._peer_timeout = 2.0
            self._peer_pulse_time = 1.0
            self._is_peer_connected = False

            self._is_master_connected = False
            self._is_peer_connected = False

            self._master_address = None
            self._peer_address = None

            self._peer_type = PEER.USER.value

            self._peers_connections = {
                f"{PEER.MASTER.value}" : {
                    "isConnected" : self._is_master_connected,
                    "address" : self._master_address
                },
                f"{PEER.USER.value}" : {
                    "isConnected" : self._is_peer_connected,
                    "address" : self._peer_address
                }
            }

            self.start()


        def OnShutdownCommand(self, msg ): 

            instructionMsg = json.loads(msg.data)
            
            peer = f"{self._peer_type}_{self.get_parameter('peer_index').value}"

            if peer in instructionMsg.keys():

                instruction = peer["exitInstruction"]

                if( instruction == EXIT_STATE.RESTART.value ):
                    self._shutdown_instruction( "sudo reboot -h now" )

                elif ( instruction == EXIT_STATE.SHUTDOWN.value ):
                    self._shutdown_instruction( "sudo shutdown -h now" )
                


        def _shutdown_instruction( self, CMD ):

            try:
                # Exécute la commande en utilisant le module subprocess
                subprocess.run(CMD, shell=True, check=True, text=True)
        
            except subprocess.CalledProcessError as e:
                self.get_logger().info(f"Une erreur s'est produite lors de l'exécution de la commande : {e}")

            finally:
                return None


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

            self._sub_master_pulse = self.create_subscription(
                String, 
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self.OnMasterPulse,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_master_pulse

            self._sub_shutdown = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.SHUTDOWN.value}",#operator_{self.get_parameter('peer_index').value}_
                self.OnShutdownCommand,
                10
            )

            self._sub_shutdown
        
            self._sub_peer = self.create_subscription(
                String, 
                f"/{PEER.DRONE.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self.OnPeerPulse,
                qos_profile=qos_profile_sensor_data
            )

            #listen for master deconnection
            
            self._sub_peer 
            self._peer_timer = self.create_timer( self._peer_timeout, self._check_peers_status )


        def _init_publishers( self ):

            self._heartbeats = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.HEARTBEAT.value,
                qos_profile=qos_profile_sensor_data
            )

            self.timer = self.create_timer( self._beat_pulsation, self._pulse)

            #watchdog publisher => send instruction when the operator connection status change
            self._pub_watchdog = self.create_publisher(
                String, 
                AVAILABLE_TOPICS.WATCHDOG.value,
                qos_profile=qos_profile_sensor_data
            )


            self._heartbeats
            self._pub_watchdog


        def _pulse( self ):
            
            if self._heartbeats is not None: 
                
                pulse_msg = String()

                info = {
                    "address" : self._address,
                    "peer" : self._peer_type
                }

                pulse_msg.data = json.dumps( info )

                self._heartbeats.publish( pulse_msg )


        def OnPeerPulse( self, pulse_msg ):

            if not self._is_peer_connected :

                json_msg = json.loads( pulse_msg.data  )

                self._is_peer_connected = True
                self._peer_address = json_msg["address"]


        def OnMasterPulse( self, pulse_msg ):

            if not self._is_master_connected :

                json_msg = json.loads( pulse_msg.data  )

                self._is_master_connected = True
                self._master_address = json_msg["address"]


        def _check_peers_status(self):

            self._peers_connections[f"{PEER.MASTER.value}" ] = {
                "isConnected" : self._is_master_connected,
                "address" : self._master_address
            }

            self._peers_connections[f"{PEER.USER.value}" ] = {
                "isConnected" : self._is_peer_connected,
                "address" : self._peer_address
            }
            
            peers_status_msg = String()
            peers_status_msg.data = json.dumps( self._peers_connections )
            
            self._pub_watchdog.publish( peers_status_msg )

            self._is_peer_connected =  False
            self._is_master_connected =  False



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