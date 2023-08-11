#!/usr/bin/env python3
########################################################################
# Filename    : __controller.py
# Description : propulsion node 
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import traceback
import json
import math
import numpy as np
import socket

#from filterpy.kalman import KalmanFilter
import subprocess
import re
import netifaces

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist

from rclpy.qos import qos_profile_sensor_data

from ..components.__microcontroller import externalBoard
from ..utils.__utils_objects import AVAILABLE_TOPICS, SENSORS_TOPICS, PEER, WIFI_INTERFACE, DIRECTION_STATE
from ..components.__audioManager import AudioManager

class OperatorNode( Node ):

        def __init__( self, **kwargs):

            super().__init__("controller", namespace=f"{PEER.USER.value}_0")

            self.EnableAudio = True
            self.VerticalAngleSwitchEnabled = False
            self.VerticalAngleSwitchTriggered = False
            self.EnableFilter = False
            
            self.forceStopFromGsc = False
            self.isDroneOutOfGameArea = False

            self._address = ""
            self._sensors_id = None

            self._pub_vel = None

            self._pub_cam = None
            self._pub_sensor = None

            self._msg_velocity = Twist()

            self.controllerOrientationMultiplier = 1 #use it to invert direction
            self.controllerPropulsionMultiplier = 1 

            self.MPU_TiltMultiplier = 1
            self.MPU_PanMultiplier = 1

            self._board = None
            self._audioManager = None

            self._propulsion_default = 40 #default percentage of thrust

            self._propulsion_max = 50
            self._propulsion_max_backward = 40
            self._propulsion = self._propulsion_default

            self._direction = 0
            self._orientation = 0

            self._steeringIncrement = 0

            self.droneDirection = 0
            
            self._filter = None

            self._delta_yaw = 0
            self._delta_roll = 0
            self._delta_pitch = 0

            self._sensor_yaw = 0
            self._sensor_pitch =0
            self._sensor_roll =0
            
            self._angleX = 0
            self._angleZ = 0

            self._obstacleInFront = False
            self._playtime = 0

            self.isGamePlayEnable = False

            self.wheelAudioTick = 0
            self.wheelAudioThreshold = 30

            self.mpu_keys = set([
                SENSORS_TOPICS.PITCH.value,
                SENSORS_TOPICS.ROLL.value,
                SENSORS_TOPICS.YAW.value,
                SENSORS_TOPICS.DELTA_PITCH.value,
                SENSORS_TOPICS.DELTA_ROLL.value,
                SENSORS_TOPICS.DELTA_YAW.value
            ]
            )

            self.pwm_fan_value = 0

            self._wifi_rssi = 0
            self._wifi_frequency = 2.4

            self.timeout_wifi_control = 60

            self._cpu_temperature = 30
            self.timeout_cpu_temperature = 30

            self.start()

        @property
        def droneRangeSensorMinThreshold(self):
            return 30

        @property
        def droneRangeSensorMaxThreshold(self):
            return 130

        @property
        def panTiltThreshold(self):
            return 10
        

        @property
        def VerticalAngleSwitchThreshold(self):
            return 40
    
        @property
        def fanspeed_stop(self):
            return 0
        
        @property
        def fanspeed_low(self):
            return 128
        
        @property
        def fanspeed_medium(self):
            return 204
        
        @property
        def fanspeed_full(self):
            return 255

        @property
        def fanspeed_gameplay( self ):
            return 240

        def start(self):

            self.get_local_ip()
            self._declare_parameters()
            self._init_publishers()
            self._init_subscribers()
            self._init_component()


        def reset( self ):

            self._propulsion = self._propulsion_default 
            self._direction = 0
            self._orientation = 0
            self._angleX = 90
            self._angleZ = 90

        def _declare_parameters( self ):
            self.declare_parameter("peer_index", 0)


        def get_local_ip( self ):

            self.get_wifi_interfaces()

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            try:

                s.connect(('8.8.8.8', 80))
                self._address = s.getsockname()[0]

            except socket.error:
                # Si la connexion échoue, nous renvoyons l'adresse IP de la machine locale
                self._address = socket.gethostbyname(socket.gethostname())
                pass

            finally:
                s.close()



        def get_wifi_interfaces( self ):
            
            wifi_interfaces = []
            interfaces = netifaces.interfaces()
            
            for interface in interfaces:

                if WIFI_INTERFACE in interface:
                    wifi_interfaces.append(interface)
            
            self._wifiInterfaces = wifi_interfaces


        def get_rssi_from_odroid_dongle(self ):

            if len( self._wifiInterfaces ) > 0:

                interface = self._wifiInterfaces[ 0 ]

                try:

                    try:
                    
                        output = subprocess.check_output(["iwconfig", interface], universal_newlines=True)
                    
                        signal_strength_match = re.search(r"Signal level=(\d+)/\d+", output)
                        frequency_match = re.search(r"Frequency:(\d+\.\d+) GHz", output)

                        if signal_strength_match and frequency_match:
                            signal_strength = int(signal_strength_match.group(1))
                            frequency = float(frequency_match.group(1))
                            
                            return (signal_strength, frequency)

                    except Exception as inner_exception:
                        self.get_logger().info(f"Erreur lors de la récupération de la puissance du signal : {inner_exception}")
                
                except Exception as outer_exception:
                    self.get_logger().info(f"Erreur lors de la récupération de la puissance du signal : {outer_exception}")
                    
            return (0,0)
        

        def _control_wifi_signal( self ):

            datas = self.get_rssi_from_odroid_dongle()
            self._wifi_rssi = datas[0] 
            self._wifi_frequency = datas[1]

        def _control_cpu_temperature( self ):
            
            if self.isGamePlayEnable is True:
      
                self.set_pwm(self.fanspeed_gameplay)

            else:

                sensors_output = subprocess.check_output("sensors", shell=True).decode()

                temp_lines = [line for line in sensors_output.split("\n") if "temp1" in line]

                if temp_lines:
   
                    temp = temp_lines[0].split("+")[1].replace("°C", "")
                    temp = temp.split()[0]
                    
                    temp = float( temp )

                    if temp is not None:

                        self._cpu_temperature = float(temp)

                        try:

                            if temp > 70.0:
                                self.set_pwm( self.fanspeed_full )  # Max fan speed
                            elif temp > 45.0:
                                self.set_pwm( self.fanspeed_medium )  # Max fan speed
                            elif temp >= 30.0:
                                self.set_pwm( self.fanspeed_low ) 
                            else:
                                self.set_pwm( self.fanspeed_stop ) 

                        except ValueError as e:
                            self.get_logger().info(f"Erreur lors de la récupération de la température du CPU : {e}")
                            self.set_pwm(self.fanspeed_low)
                
                    else:
                
                        self.set_pwm(self.fanspeed_low)
    

        def set_pwm(self, value):
            
            if value != self.pwm_fan_value:

                self.pwm_fan_value = value

                cmd = f"sudo /usr/local/bin/trafalgar_set_pwm {value}"
                subprocess.run(cmd, shell=True, check=True)


        def _init_component(self):
            
            #if self.EnableFilter is True:
            #    self._init_filter()

            self._board = externalBoard( self._board_datas )
            self.reset()

            self._board._enable()

            if self.EnableAudio is True: 
                self._audioManager = AudioManager()
                self._audioManager._enable()

            self._init_timers()

        def _init_timers( self ):

            self.create_timer(self.timeout_cpu_temperature, self._control_cpu_temperature )
            self.create_timer(self.timeout_wifi_control, self._control_wifi_signal ) 


        def _init_publishers( self ):

            self._pub_vel  = self.create_publisher(
                Twist, 
                AVAILABLE_TOPICS.VELOCITY.value,
                qos_profile=qos_profile_sensor_data
            )
            
            self._pub_vel 

            self._pub_cam = self.create_publisher(
                Vector3,
                AVAILABLE_TOPICS.PANTILT.value,
                qos_profile=qos_profile_sensor_data
            )

            self._pub_cam


            self._pub_sensor = self.create_publisher(
                String,
                AVAILABLE_TOPICS.SENSOR.value,
                qos_profile = qos_profile_sensor_data
            )

            self._pub_sensor   


        def _init_subscribers( self ):
            
            self._sub_master = self.create_subscription(
                String,
                f"/{PEER.MASTER.value}/{AVAILABLE_TOPICS.HEARTBEAT.value}",
                self.OnMasterPulse,
                qos_profile=qos_profile_sensor_data
            )
            
            self._sub_master  # prevent unused variable warning

            self._sub_drone_sensor = self.create_subscription(
                String,
                f"/{PEER.DRONE.value}_{self.get_parameter('peer_index').value}/{AVAILABLE_TOPICS.SENSOR.value}",
                self.OnDroneDatas,
                qos_profile=qos_profile_sensor_data
            )

            self._sub_drone_sensor   
            

        def _update_direction( self, updateDirection = DIRECTION_STATE.STOP.value ):
            
            #self.get_logger().info( f"update direction : {updateDirection}")

            updateDirection = int(np.clip( updateDirection, DIRECTION_STATE.BACKWARD.value, DIRECTION_STATE.FORWARD.value ))

            if self.forceStopFromGsc is False and self.isDroneOutOfGameArea is False:

                self._direction = updateDirection
            
                if( self._direction == DIRECTION_STATE.STOP.value):
                    self.reset()
                
                self._send_controller_cmd()
            
            """
                else :

                if self._direction != DIRECTION_STATE.STOP.value: 

                    if self._audioManager is not None:
                        self._audioManager.gameplayMusic( self.isGamePlayEnable, DIRECTION_STATE.STOP.value )
            
            """



        def _updateWheelAudio( self, increment ): 

            self.wheelAudioTick += abs(increment)

            if self.wheelAudioTick >= self.wheelAudioThreshold: 
                self.wheelAudioTick = 0

                if( self.isGamePlayEnable is True ):
                    if self._audioManager is not None:
                        self._audioManager.play_sfx("wheel")


        def _update_panoramic( self, pan = 90, tilt=90):

            update_tilt = tilt
            update_pan = pan

            vec = Vector3()
            
            vec.y = float( update_tilt )
            vec.z = float( update_pan )

            self._pub_cam.publish( vec )


        def _board_datas( self, json_datas ): 
            
            datas = json_datas

            if SENSORS_TOPICS.ORIENTATION.value in datas:
                self.OnNewOrientation(datas[SENSORS_TOPICS.ORIENTATION.value])
            
            if SENSORS_TOPICS.PROPULSION.value in datas:
                self.OnNewPropulsion(datas[SENSORS_TOPICS.PROPULSION.value])

            if SENSORS_TOPICS.SHORT_PRESS.value in datas and SENSORS_TOPICS.LONG_PRESS.value in datas:
                self.OnButtonPress( 
                    datas[SENSORS_TOPICS.SHORT_PRESS.value], 
                    datas[SENSORS_TOPICS.LONG_PRESS.value] 
                )

            if self.mpu_keys.issubset( datas.keys()):   
                self.OnMPUDatas(
                    pitch=datas[SENSORS_TOPICS.PITCH.value],
                    roll=datas[SENSORS_TOPICS.ROLL.value],
                    yaw=datas[ SENSORS_TOPICS.YAW.value ],
                    delta_p = datas[ SENSORS_TOPICS.DELTA_PITCH.value ],
                    delta_r=datas[ SENSORS_TOPICS.DELTA_ROLL.value ],
                    delta_y=datas[ SENSORS_TOPICS.DELTA_YAW.value ]
                )

            self._send_sensors_datas(datas)


        def OnNewOrientation( self, increment ):

            if increment != 0:
                
                #self.get_logger().info( f"update orientation : {increment}")

                self._steeringIncrement = int(increment * self.controllerOrientationMultiplier )
                self._send_controller_cmd()
                
                self._updateWheelAudio( increment )


        def OnNewPropulsion( self, updateLevelIncrement ): 
            
            #self.get_logger().info( f"update propulsion : {updateLevelIncrement}")

            if updateLevelIncrement != 0 and self._direction != DIRECTION_STATE.STOP.value:
            
                update_propulsion = self._propulsion + (updateLevelIncrement/5) 
    
                if self._direction >= 0:

                    update_propulsion = math.floor( int(np.clip( update_propulsion, self._propulsion_default, self._propulsion_max ) ) )

                else :

                    update_propulsion = math.floor( int(np.clip( update_propulsion, self._propulsion_default, self._propulsion_max_backward ) ) )


                update_propulsion = int(np.clip(update_propulsion * self.controllerPropulsionMultiplier, self._propulsion_default, self._propulsion_max) )
            
                if update_propulsion != self._propulsion:

                    self._propulsion = update_propulsion
                    self._send_controller_cmd()
    

        def OnButtonPress( self, shortPress = False, longPress = False ):
            
            if longPress is True:

                if self._direction != DIRECTION_STATE.BACKWARD.value:
                    self._update_direction(DIRECTION_STATE.BACKWARD.value)
                    
                    if( self.isGamePlayEnable is True ):
                        if self._audioManager is not None:
                            self._audioManager.play_sfx("bell")

            if shortPress is True:

                if self._direction == DIRECTION_STATE.STOP.value:
                    self._update_direction(DIRECTION_STATE.FORWARD.value)
                else:
                    self._update_direction(DIRECTION_STATE.STOP.value)
                
                if( self.isGamePlayEnable is True ):  
                    if self._audioManager is not None:
                        self._audioManager.play_sfx("bell")


        def OnMPUDatas( self, pitch=0, roll=0, yaw=0, delta_p = 0, delta_r=0, delta_y=0 ):
            
            #if self._filter is not None:
            #    pitch, roll, yaw = self.smooth_imu_data(pitch, roll, yaw)
            
            self._sensor_yaw = yaw
            self._sensor_pitch = pitch
            self._sensor_roll = roll

            self._delta_p = delta_p
            self._delta_r =  delta_r
            self._delta_y = delta_y

            #angle_aroundX = np.clip(90 - yaw, 0, 180)
            #angle_aroundY = np.clip( 90 - delta_y, 0,180 )
            #angle_aroundZ = np.clip( 90 + delta_p, 0,180 )
            
            
            angleX = int(np.clip(90 + roll * self.MPU_TiltMultiplier , 0, 180) )
            angleZ = int(np.clip( self._angleZ * self.MPU_PanMultiplier + delta_p, 0,180 ))

            if abs(angleX - self._angleX ) >= self.panTiltThreshold or abs(angleZ - self._angleZ) >= self.panTiltThreshold:
           
                self._angleX = angleX
                self._angleZ = angleZ

                self._CheckVerticalAngleSwitchStatus( angleX )
                
                self._update_panoramic(pan=self._angleZ, tilt = self._angleX  )


        def _CheckVerticalAngleSwitchStatus( self, angle ):

            if angle < self.VerticalAngleSwitchThreshold:
                self.VerticalAngleSwitchTriggered = True
            else:
                self.VerticalAngleSwitchTriggered = False
            
            if self.VerticalAngleSwitchEnabled is True:
                self.get_logger().info( "vertical angle switch is triggered !")


        def _send_controller_cmd( self ):

            """
            In Unity, the X axis points right, Y up, and Z forward. 
            ROS, on the other hand, supports various coordinate frames: 
            in the most commonly-used one, X points forward, Y left, and Z up. In ROS terminology, 
            this frame is called "FLU" (forward, left, up), 
            whereas the Unity coordinate frame would be "RUF" (right, up, forward).
            """

            if self._msg_velocity is not None:
                    
                self._msg_velocity.linear.x = float(int(self._direction) * int(self._propulsion))
                self._msg_velocity.angular.z = float( int(self._steeringIncrement) )

                self._pub_vel.publish( self._msg_velocity )


        def _send_sensors_datas( self, sensor_json ):
            
            sensor_msg = String()
            
            if self._sensors_id is None:
                self._sensors_id = self.get_parameter("peer_index").value
            

            sensor_json[SENSORS_TOPICS.IP.value] = f"{self._address}"
            sensor_json[SENSORS_TOPICS.WIFI.value] = ( self._wifi_rssi, self._wifi_frequency )
     
            #self.get_logger().info(sensor_json[f"{SENSORS_TOPICS.IP}"] )
            sensors_datas = {
                SENSORS_TOPICS.INDEX.value : self._sensors_id,
                SENSORS_TOPICS.DATAS.value : sensor_json
            }
            sensor_msg.data = json.dumps( sensors_datas )

            self._pub_sensor.publish( sensor_msg )
        

        def OnDroneDatas( self, msg ):
            
            drone_sensors = json.loads( msg.data )

            if( SENSORS_TOPICS.OFF_AREA.value in drone_sensors ):

                self.isDroneOutOfGameArea = drone_sensors[SENSORS_TOPICS.OFF_AREA.value]

                if self.isDroneOutOfGameArea is True and self._direction != DIRECTION_STATE.STOP.value:
                    self._update_direction(DIRECTION_STATE.STOP.value)


            if( SENSORS_TOPICS.INDEX.value in drone_sensors and SENSORS_TOPICS.DATAS.value in drone_sensors ):
    
                sensor_datas = drone_sensors[SENSORS_TOPICS.DATAS.value]

                for topic in sensor_datas:

                    if( topic == SENSORS_TOPICS.DIRECTION.value ):

                        updateDroneDirection = sensor_datas[topic] 

                        if updateDroneDirection != self.droneDirection:

                            self.droneDirection = updateDroneDirection

                            if self._audioManager is not None :
                                self._audioManager.gameplayMusic( self.isGamePlayEnable, updateDroneDirection )
                                
                                self.get_logger().info( f"music has changed on drone value user direction : {self._direction} / drone direction : {self.droneDirection} / obsacle in front : {self._obstacleInFront} ")


                    elif(topic == SENSORS_TOPICS.OBSTACLE_DISTANCE.value  ):

                        sensor_obstacle = sensor_datas[topic]

                        if sensor_obstacle < self.droneRangeSensorMaxThreshold and sensor_obstacle >= self.droneRangeSensorMinThreshold: 
                            self._obstacleInFront = True
                        else:
                            self._obstacleInFront = False
            


            if self.droneDirection != self._direction : 

                if self._obstacleInFront is False: 
                    self._update_direction( self._direction )


        def OnMasterPulse( self, msg ):

            master_pulse = json.loads( msg.data )
            
            if "peers" in master_pulse: 

                peers = master_pulse["peers"]
                peerUpdate = f"peer_{self.get_parameter('peer_index').value}"

                if peerUpdate in peers: 

                    statusUpdate = peers[peerUpdate]

                    if "stop" in statusUpdate: 

                        self.forceStopFromGsc = statusUpdate["stop"]

                        if self.forceStopFromGsc is True and self._direction != DIRECTION_STATE.STOP.value:
                            self._update_direction(DIRECTION_STATE.STOP.value)
                    
                    if "playtime" in statusUpdate:
                        self._playtime =  statusUpdate["playtime"]

                    if "enable" in statusUpdate:
                        
                        enableUpdate = statusUpdate["enable"]

                        if enableUpdate is True and self.isGamePlayEnable is False:

                            self._update_direction(DIRECTION_STATE.STOP.value)

                            if self._audioManager is not None:
                                self._audioManager.gameplayMusic( enableUpdate, 0 )
                        
                        self.isGamePlayEnable = enableUpdate 
                        
                        
                    else:

                        self.isGamePlayEnable = False 
                        
                        self.standard_reset()

            else:

                self.isGamePlayEnable = False 

                self.standard_reset()




        def standard_reset( self ):

            if self._direction != DIRECTION_STATE.STOP.value:   
                self._update_direction(DIRECTION_STATE.STOP.value)
                        
                if self._audioManager is not None:
                    self._audioManager.gameplayMusic( self.isGamePlayEnable, self._direction )

            if self.isGamePlayEnable is False:
                self._audioManager.stop_music( )  

        def exit(self):

            self.get_logger().info("shutdown controller")   



def main(args=None):

    rclpy.init(args=args)

    controller_node = None 

    try:

        controller_node = OperatorNode()

        rclpy.spin( controller_node )

    except Exception as e:
        traceback.print_exc()

    except KeyboardInterrupt:
        print("user force interruption")
        
    finally:

        if controller_node is not None:
            controller_node.exit()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()


"""
        def _init_filter( self ): 
            
            self._filter = KalmanFilter(dim_x=3, dim_z=3)

            if self._filter is not None:

                self._filter.F = np.eye(3)  # Matrice de transition, généralement identité
                self._filter.H = np.eye(3)  # Matrice de mesure, généralement identité
                
                # Initialisez les variables du filtre de Kalman
                initial_state = np.array([0, -90, 0])  # Initialisez l'état initial
                self._filter.x = initial_state
                self._filter.P *= 0.1  # Matrice de covariance de l'état initial
       

        def smooth_imu_data(self, pitch, roll, yaw):
            # Mettez à jour le filtre de Kalman avec les nouvelles mesures IMU
            self._filter.predict()
            self._filter.update(np.array([pitch, roll, yaw]))

            # Obtenez les valeurs filtrées de l'état du système
            filtered_state = self._filter.x

            return filtered_state[0], filtered_state[1], filtered_state[2]
"""