#!/usr/bin/env python3
########################################################################
# Filename    : __imu_view.py
# Description : read mpu data to drive a PAN TILT camera
# Author      : Man'O'AR
# modification: 22/04/2023
########################################################################
import math
from threading import Thread, Lock
from time import sleep
from smbus2 import SMBUS

from sensor_msgs.msg import Imu

class viewMPU( object ):
    def __init__(
            self, 
            master, 
            address = 0x68,
            verbose = False ):
        
        super().__init__()

        self._master = master
        self._verbose = verbose

        self._address = address
        self._bus = None

        self._roll = 0 
        self._pitch = 0,
        self._yaw = 0
        self._temp = 0

        self._lock = Lock()
        self._thread_stopped = False
        self._thread_ = None


    @property
    def who_am_i_reg(self):
        # Adresse I2C du registre "WHO_AM_I"
        return 0x75
    
    @property
    def sample_rate_div_reg(self):
        # Adresse I2C du registre de configuration de l'échantillonnage
        return 0x19

    @property
    def gyro_config_reg(self):
        # Adresse I2C du registre de configuration des gyromètres
        return 0x1B

    @property
    def accel_config_reg(self):
        # Adresse I2C du registre de configuration des accéléromètres
        return  0x1C
    
    @property
    def mag_config_reg(self):
        # Adresse I2C du registre de configuration du magnétomètre
        return  0x0A

    @property
    def data_reg(self):
        # Adresse I2C du registre de lecture de données
        return  0x3B

    @property
    def mag_declination( self ):
        return -0.0123
    

    def start( self ):

        self._bus = SMBus(1)
        
        # Lecture de l'identifiant du périphérique MPU9250 (doit renvoyer 0x71)
        who_am_i = self._bus.read_byte_data( self._address, self.who_am_i_reg )
        
        if who_am_i != 0x71:
            print("Erreur : identifiant du périphérique invalide")

        # Configuration de l'échantillonnage des données
        self._bus.write_byte_data( self._address, self.sample_rate_div_reg, 0x07 )   # fréquence d'échantillonnage = 1 kHz
        self._bus.write_byte_data( self._address, self.gyro_config_reg, 0x18 )       # échelle de mesure des gyromètres = ±2000 degrés/seconde
        self._bus.write_byte_data( self._address, self.accel_config_reg, 0x18 )      # échelle de mesure des accéléromètres = ±16 g
        self._bus.write_byte_data( self._address, self.mag_config_reg, 0x16 )  

        self._thread_ = Thread( target=self._run )
        self._thread_.daemon = True
        self._thread_.start()


    def calculate_orientation_and_inclination(self, ax, ay, az, mx, my):

        # Calcul de l'orientation
        roll = math.atan2( ay, az )
        pitch = math.atan2( -ax, math.sqrt( ay*ay + az*az ) )

        # Correction des valeurs du magnétomètre en fonction de la déclinaison magnétique
        mx_corr = mx * math.cos( self.mag_declination ) - my * math.sin( self.mag_declination )
        my_corr = mx * math.sin( self.mag_declination ) + my * math.cos( self.mag_declination )

        # Calcul de l'orientation magnétique
        yaw = math.atan2( my_corr, mx_corr )

        # Correction de l'orientation pour éviter les angles négatifs
        if yaw < 0:
            yaw += 2 * math.pi

        # Conversion des angles en degrés
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return roll, pitch, yaw

    def read( self ):
        
        if self._bus is not None:
            
            data = self._bus.read_i2c_block_data( self._address, self.data_reg, 14 )
            
            # Conversion des données brutes en valeurs de capteur
            ax = (data[0] << 8) | data[1]
            ay = (data[2] << 8) | data[3]
            az = (data[4] << 8) | data[5]
            
            temp = (data[6] << 8) | data[7]
            
            gx = (data[8] << 8) | data[9]
            gy = (data[10] << 8) | data[11]
            gz = (data[12] << 8) | data[13]            

            # Conversion des valeurs de capteur en unités physiques
            accel_scale_factor = 16.0 / 32768.0  # échelle de mesure des accéléromètres = ±16 g
            gyro_scale_factor = 2000.0 / 32768.0  # échelle de mesure des gyromètres = ±2000 degrés/seconde
            mag_scale_factor = 4912.0 / 32760.0  # échelle de mesure du magnétomètre = ±4912 microtesla

            ax = ax * accel_scale_factor
            ay = ay * accel_scale_factor
            az = az * accel_scale_factor
    
            gx = gx * gyro_scale_factor
            gy = gy * gyro_scale_factor
            gz = gz * gyro_scale_factor

            # Lecture des données du magnétomètre
            self._bus.write_byte_data( self._address, 0x37, 0x02 )  # demande de lecture des données du magnétomètre
            sleep(0.1)  # pause de 100 ms pour permettre au magnétomètre de prendre les mesures
            mag_data = self._bus.read_i2c_block_data( self._address, 0x03, 6 )

            # Conversion des données du magnétomètre en unités physiques
            mx = (mag_data[1] << 8) | mag_data[0]
            my = (mag_data[3] << 8) | mag_data[2]
            mz = (mag_data[5] << 8) | mag_data[4]
            mx = mx * mag_scale_factor
            my = my * mag_scale_factor
            mz = mz * mag_scale_factor

            self._roll, self._pitch, self._yaw = self.calculate_orientation_and_inclination(
            ax, 
            ay, 
            az, 
            mx, 
            my, 
            mz
            )

            self._temp = temp / 333.87 + 21  # conversion de la température en degrés Celsius


    def send( self ):

        if self._master is not None:

            self._master._update_pan( )
            self._master._update_tilt( )

    def _run( self ):
        
        while not self._thread_stopped:

            try:
                
                with self._lock:

                    self.read()
                    
                    self.send()
                

            except Exception as ex:
                print( f"an error has occured in videocapture : {ex}" )
                break 
        

    def exit( self ):

        if self._thread_ is not None:

            self._thread_stopped  = True
            self._thread_.join()
            sleep( 1 )
            self._thread_ = None