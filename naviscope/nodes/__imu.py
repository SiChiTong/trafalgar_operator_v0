#!/usr/bin/env python3
########################################################################
# Filename    : __imu.py
# Description : imu node 
# Author      : Man'O'AR
# modification: 22/04/2023
########################################################################
import sys
from time import sleep
import math
import smbus2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from ..utils.__utils_objects import AVAILABLE_TOPICS, OPERATOR

# MPU9250 Register Map
MPU_ADDRESS = 0x68
# Accelerometer and Gyroscope Registers
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40
TEMP_OUT_H = 0x41
TEMP_OUT_L = 0x42
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

# Magnetometer Registers
MAG_ADDRESS = 0x0C
MAG_XOUT_L = 0x03  # register address for the magnetometer X-axis low byte
MAG_YOUT_L = 0x05  # register address for the magnetometer Y-axis low byte
MAG_ZOUT_L = 0x07  # register address for the magnetometer Z-axis low byte
MAG_XOUT_H = 0x04
MAG_YOUT_H = 0x06
MAG_ZOUT_H = 0x08
MAG_CNTL = 0x0A
MAG_PWR_DOWN = 0x00
MAG_CNT_MEAS1 = 0x12
MAG_CNT_MEAS2 = 0x16
MAG_FUSE_ROM = 0x0F
MAG_CNTL2 = 0x0B
MAG_RESET = 0x01

#CONSTANTS
GYRO_SCALE_FACTOR = 131.0  # gyroscope scale factor for 250 degrees/s
ACC_SCALE_FACTOR_X = 16384.0
ACC_SCALE_FACTOR_Y = 16384.0
ACC_SCALE_FACTOR_Z = 16384.0
MAG_SCALE_FACTOR_X = 0.6  # magnetometer scale factor for X-axis
MAG_SCALE_FACTOR_Y = 0.6  # magnetometer scale factor for Y-axis
MAG_SCALE_FACTOR_Z = 0.6  # magnetometer scale factor for Z-axis
STANDARD_GRAVITY = 9.80665  # standard acceleration due to gravity in m/s^2
#ACCEL_SCALE_FACTOR_2G

class ImuNode( Node ):

    def __init__(self, EnableRAWImuPub = False ):

        super().__init__("imu", namespace="operator_0")

        self._enableRawPublisher = EnableRAWImuPub

        self._pub_imu = None
        self._pub_pantilt = None

        self._timer = None
        self._timer_tick = 1/10

        self.mag_x_offset = 0.0 # Magnetometer X-axis offset
        self.mag_y_offset = 0.0 # Magnetometer Y-axis offset
        self.mag_z_offset = 0.0 # Magnetometer Z-axis offset

        self._panTilt_msg = Vector3()
        self._imu_msg = Imu()

        self._bus = None

        self._roll = 0 
        self._pitch = 0
        self._yaw = 0

        self.accel_bias = (0, 0, 0)
        self.gyro_bias = (0, 0, 0)
        self.mag_bias = (0, 0, 0)

        self.start()
    
    def start(self):
            
            self._declare_parameters()

            self._initialize_bus()
            self._init_publishers()


    def _initialize_bus( self ): 

        self._bus = smbus2.SMBus(1) # Set the correct I2C bus number
        
        self._bus.write_byte_data(MPU_ADDRESS, 0x6B, 0x00)  # Wake up MPU9250
        self._bus.write_byte_data(MPU_ADDRESS, 0x1B, 0x18)  # Set full scale range for gyroscope
        self._bus.write_byte_data(MPU_ADDRESS, 0x1C, 0x08)  # Set full scale range for accelerometer
        self._bus.write_byte_data(MPU_ADDRESS, 0x37, 0x02)  # Set bypass mode for magnetometer
        
        self._bus.write_byte_data(MAG_ADDRESS, MAG_CNTL, MAG_CNT_MEAS1)
        
        sleep(0.1)


    def _declare_parameters( self ):

        self.declare_parameter("verbose", False)
        self.declare_parameter("peer_index", 0)    


    def _init_publishers( self ):

        self._pub_pantilt  = self.create_publisher(
            Vector3, 
            AVAILABLE_TOPICS.PANTILT.value,
            10
        )
            
        self._pub_pantilt

        if self._enableRawPublisher is True:

            self._pub_imu  = self.create_publisher(
                Imu, 
                AVAILABLE_TOPICS.IMU.value,
                10
            )
            
            self._pub_imu
         
        
        self.timer = self.create_timer( self._timer_tick, self.publish_imu_data )


    def read_accelerometer( self ):
            
        x = self.read_word_2c(ACCEL_XOUT_L)  
        y = self.read_word_2c(ACCEL_YOUT_L) 
        z = self.read_word_2c(ACCEL_ZOUT_L) 

        x -= self.accel_bias[0]
        y -= self.accel_bias[1]
        z -= self.accel_bias[2]

        x *= 1 / ACC_SCALE_FACTOR_X
        y *= 1 / ACC_SCALE_FACTOR_Y
        z *= 1 / ACC_SCALE_FACTOR_Z

        return x, y, z


    def read_gyroscope( self ):

        x = self.read_word_2c(GYRO_XOUT_H)
        y = self.read_word_2c(GYRO_YOUT_H)
        z = self.read_word_2c(GYRO_ZOUT_H) 

        x -= self.gyro_bias[0]
        y -= self.gyro_bias[1]
        z -= self.gyro_bias[2]

        x *= 1 / GYRO_SCALE_FACTOR
        y *= 1 / GYRO_SCALE_FACTOR
        z *= 1 / GYRO_SCALE_FACTOR

        return x, y, z
    

    def read_magnetometer( self ):

        x = self.read_word_2c(MAG_XOUT_L) * MAG_SCALE_FACTOR_X
        y = self.read_word_2c(MAG_YOUT_L) * MAG_SCALE_FACTOR_Y
        z = self.read_word_2c(MAG_ZOUT_L) * MAG_SCALE_FACTOR_Z
        
        x -= self.mag_bias[0]
        y -= self.mag_bias[1]
        z -= self.mag_bias[2]
        
        x *= MAG_SCALE_FACTOR_X
        y *= MAG_SCALE_FACTOR_Y
        z *= MAG_SCALE_FACTOR_Z

        return x, y, z


    def calibrate_sensors(self):
        # Put the IMU in a stable and known position
        # and read the raw sensor values for a period of time
        x_accel = []
        y_accel = []
        z_accel = []
        
        x_gyro = []
        y_gyro = []
        z_gyro = []
        
        x_mag = []
        y_mag = []
        z_mag = []

        for i in range(1000):

            x_accel.append(self.read_word_2c(ACCEL_XOUT_L))
            y_accel.append(self.read_word_2c(ACCEL_YOUT_L))
            z_accel.append(self.read_word_2c(ACCEL_ZOUT_L))

            x_gyro.append(self.read_word_2c(GYRO_XOUT_L))
            y_gyro.append(self.read_word_2c(GYRO_YOUT_L))
            z_gyro.append(self.read_word_2c(GYRO_ZOUT_L))

            x_mag.append(self.read_word_2c(MAG_XOUT_L))
            y_mag.append(self.read_word_2c(MAG_YOUT_L))
            z_mag.append(self.read_word_2c(MAG_ZOUT_L))

            sleep(0.01)

        # Calculate the mean value for each axis
        accel_bias = (-sum(x_accel) / len(x_accel), -sum(y_accel) / len(y_accel), -sum(z_accel) / len(z_accel))
        gyro_bias = (-sum(x_gyro) / len(x_gyro), -sum(y_gyro) / len(y_gyro), -sum(z_gyro) / len(z_gyro))
        mag_bias = (-sum(x_mag) / len(x_mag), -sum(y_mag) / len(y_mag), -sum(z_mag) / len(z_mag))

        # Set the bias values in the instance variables
        self.accel_bias = accel_bias
        self.gyro_bias = gyro_bias
        self.mag_bias = mag_bias


    def read_word_2c(self, reg):

        high = self._bus.read_byte_data(MPU_ADDRESS, reg)
        low = self._bus.read_byte_data(MPU_ADDRESS, reg + 1)

        val = (high << 8) + low
 
        if val >= 0x8000:

            return -((65535 - val) + 1)
        
        else:

            return val


    def publish_imu_data(self):
        # Read Accelerometer and Gyroscope data
        accel_xout, accel_yout, accel_zout = self.read_acclerometer()
        gyro_xout, gyro_yout, gyro_zout = self.read_gyroscope()
        mag_xout, mag_yout, mag_zout = self.read_magnetometer()

        mag_heading = math.atan2(mag_yout, mag_xout)
 
        if mag_heading < 0:
            mag_heading += 2 * math.pi
 
        if mag_heading > 2 * math.pi:
            mag_heading -= 2 * math.pi
 
        # Conversion des angles en degrés
        self._roll = math.atan2(accel_yout, accel_zout)
        self._pitch = math.atan2(-accel_xout, math.sqrt(accel_yout ** 2 + accel_zout ** 2))
        self._yaw = math.degrees(mag_heading)  

        self._panTilt_msg.x = self._pitch
        self._panTilt_msg.y = self._roll
        self._panTilt_msg.z = self._yaw

        if self.get_parameter("verbose").value is True:
            self.get_logger().info(f"Accelerometer bias: {self.accel_bias}")
    
        self._pub_pantilt.publish( self._panTilt_msg )

        # Publish IMU data
        self._imu_msg.header.stamp = self.get_clock().now().to_msg()
 
        self._imu_msg.orientation.w = math.cos(self._roll / 2) * math.cos(self._pitch / 2) * math.cos(self._yaw / 2) + math.sin(self._roll / 2) * math.sin(self._pitch / 2) * math.sin(self._yaw / 2)
        self._imu_msg.orientation.x = math.sin(self._roll / 2) * math.cos(self._pitch / 2) * math.cos(self._yaw / 2) - math.cos(self._roll / 2) * math.sin(self._pitch / 2) * math.sin(self._yaw / 2)
        self._imu_msg.orientation.y = math.cos(self._roll / 2) * math.sin(self._pitch / 2) * math.cos(self._yaw / 2) + math.sin(self._roll / 2) * math.cos(self._pitch / 2) * math.sin(self._yaw / 2)

        self._imu_msg.angular_velocity.x = gyro_xout
        self._imu_msg.angular_velocity.y = gyro_yout
        self._imu_msg.angular_velocity.z = gyro_zout
 
        self._imu_msg.linear_acceleration.x = accel_xout * STANDARD_GRAVITY
        self._imu_msg.linear_acceleration.y = accel_yout * STANDARD_GRAVITY
        self._imu_msg.linear_acceleration.z = accel_zout * STANDARD_GRAVITY
 
        if self._enableRawPublisher is True: 
            self._pub_imu.publish( self._imu_msg )



def main(args=None):

    rclpy.init(args=args)

    imu_node = None 

    try:

        imu_node  = ImuNode()

        rclpy.spin( imu_node  )

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

        if imu_node is not None:
            imu_node.exit()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()