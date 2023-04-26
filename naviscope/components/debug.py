#!/usr/bin/env python3
########################################################################
# Filename    : __rot_orientation.py
# Description : read rotary encoder data to drive the boat
# Author      : Man'O'AR
# modification: 22/04/2023
########################################################################
import numpy as np
import odroid_wiringpi as wiringpi
from time import sleep

class rotaryEncoder( object ):


    def __init__(
            self, 
            pin_out,
            callback,
            enableRange = False,
            resetToMin = True,
            minClip = 50,
            maxClip = 100,
            increment= 10,
            verbose = False ):
        
        super().__init__()

        self._verbose = verbose
        
        self._clk = pin_out["clk"]
        self._dt = pin_out["dt"]
        self._sw = pin_out["sw"]
        
        self._increment_factor = increment
        self._cb = callback

        self._previous_state = None
        
        self._enableRange = enableRange
        self._resetToMin = resetToMin
        self._min_value = minClip
        self._max_value = maxClip
        self._center = (minClip + maxClip) / 2
        self._cursor = 0

        self.enable()


    def enable( self ):
        
        wiringpi.wiringPiSetup()

        wiringpi.pinMode( self._clk, wiringpi.INPUT )
        wiringpi.pinMode( self._dt, wiringpi.INPUT )

        wiringpi.pullUpDnControl( self._clk, wiringpi.PUD_DOWN )
        wiringpi.pullUpDnControl( self._dt, wiringpi.PUD_DOWN )

        self._previous_state = wiringpi.digitalRead( self._clk )

        wiringpi.wiringPiISR( self._clk, wiringpi.INT_EDGE_BOTH, self. onEncoderTriggered )

        self.reset()


    def reset( self ):

        if self._enableRange is True: 

            if self._resetToMin is True: 

                self._cursor = self._min_value

            else:

                self._cursor = self._center


    def onEncoderTriggered( self ):

        clk_state = wiringpi.digitalRead( self._clk ) 
        dt_state = wiringpi.digitalRead( self._dt )

        if clk_state != self._previous_state : 
            
            if dt_state != clk_state:
                
                self._cursor += self._increment_factor
                self._cursor = np.clip( self._cursor, self._min_value, self._max_value )
                
                if self._enableRange is True: 
                                  
                    self._cb( self._cursor )
  
                else: 
                                        
                    self._cb( self._increment_factor )

            else:

                self._cursor -= self._increment_factor
                self._cursor = np.clip( self._cursor, self._min_value, self._max_value )

                if self._enableRange is True: 
                                  
                    self._cb( self._cursor )
  
                else: 
                                        
                    self._cb( -self._increment_factor )
    
        self._previous_state = clk_state
    


    def disable( self ):
        print("orientation encoder is disabled")


pinouts = {
    "dt" : 5,
    "clk" : 4,
    "sw":0
}


def printValues( data ):
    print( data )



comp = rotaryEncoder(
    pin_out = pinouts,
    callback = printValues
)


while True: 
    print("do smthg ")
    sleep(2)

