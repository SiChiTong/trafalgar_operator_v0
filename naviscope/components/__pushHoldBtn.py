#!/usr/bin/env python3
########################################################################
# Filename    : __btn_propulsion.py
# Description : read button state to drive propulsion 
# Author      : Man'O'AR
# modification: 22/04/2023
########################################################################

import odroid_wiringpi as wiringpi
from time import monotonic

class pushHoldButton( object ):
    def __init__(
            self, 
            pin_out,
            callback,
            hold_threshold = 3, 
            verbose = False ):
        
        super().__init__()

        self._verbose = verbose

        self._pin = pin_out
        self._cb = callback

        self._isPressed = False
        self._isLongPressed = False

        self._hold_threshold = hold_threshold

        self._press_start = 0
        self._press_duration = 0

        self._propulsion_is_on_standby = True

        self.enable()

    def enable( self ):
        
        wiringpi.wiringPiSetup()
        
        wiringpi.pinMode( self._pin, wiringpi.INPUT )
        wiringpi.pullUpDnControl( self._pin, wiringpi.PUD_UP )


    def send_stp( self ):
        
        self._propulsion_is_on_standby = True
        self._isLongPressed = False

        if self._cb is not None:
            self._cb._update_propulsion( 0 )


    def send_fwd( self ):
        
        self._propulsion_is_on_standby = False
        self._isLongPressed = False

        if self._cb is not None:
            self._cb._update_propulsion( 1 )


    def send_bwd( self ):

        self._isLongPressed = True
        self._propulsion_is_on_standby = False

        if self._cb is not None:
            self._cb._update_propulsion( -1 )



    def _read_state( self ):

        # Read button state
        state = wiringpi.digitalRead( self._pin )

        # If button is pressed
        if state == wiringpi.LOW and not self._isPressed:
            self._press_start =  monotonic()
            self._isPressed = True

        # Calculate press duration
        if self._isPressed is True:
            self._press_duration = monotonic() - self._press_start

        # If button is released
        if state == wiringpi.HIGH and self._isPressed:

            if self._press_duration <= self._hold_threshold :

                if self._propulsion_is_on_standby is True:
                    
                    self.send_fwd()

                else:
                    
                    self.send_stp()

            else:

                self.send_stp()
    
            self._isPressed = False

        if self._isPressed and self._press_duration > self._hold_threshold :

            if self._press_duration > self._hold_threshold:

                if self._isLongPressed is False: 
                    self.send_bwd()

    def disable( self ):
        print("propulsion button is disabled")