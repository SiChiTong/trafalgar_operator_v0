#!/usr/bin/env python3
########################################################################
# Filename    : __rot_orientation.py
# Description : read rotary encoder data to drive the boat
# Author      : Man'O'AR
# modification: 22/04/2023
########################################################################

import odroid_wiringpi as wiringpi

class OrientationEncoder( object ):
    def __init__(
            self, 
            master, 
            pin_clk = 4,
            pin_dt =  5,
            verbose = False ):
        
        super().__init__()

        self._master = master
        self._verbose = verbose
        
        self._clk = pin_clk
        self._dt = pin_dt

        self._previous_state = None

    @property
    def increment_factor(self):
        return 10
    
    def start( self ):
        
        wiringpi.wiringPiSetup()

        wiringpi.pinMode( self._clk, wiringpi.INPUT )
        wiringpi.pinMode( self._dt, wiringpi.INPUT )

        wiringpi.pullUpDnControl( self._clk, wiringpi.PUD_DOWN )
        wiringpi.pullUpDnControl( self._dt, wiringpi.PUD_DOWN )

        self._previous_state = wiringpi.digitalRead( self._clk )

        wiringpi.wiringPiISR( self._clk, wiringpi.INT_EDGE_BOTH, self. onEncoderTriggered )


    def send_cw( self ):

        if self._master is not None:
            self._master._update_orientation( increment = self.increment_factor )


    def send_ccw( self ):

        if self._master is not None:
            self._master._update_orientation( increment = -self.increment_factor )


    def onEncoderTriggered( self ):

        clk_state = wiringpi.digitalRead( self._clk ) 
        dt_state = wiringpi.digitalRead( self._dt )

        if clk_state != self._previous_state : 
            
            if dt_state != clk_state:

                self.send_cw()

            else:

                self.send_ccw()
    
        self._previous_state = clk_state
    

    def exit( self ):
        print("orientation encoder is disabled")