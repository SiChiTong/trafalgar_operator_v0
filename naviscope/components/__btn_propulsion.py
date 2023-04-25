#!/usr/bin/env python3
########################################################################
# Filename    : __btn_propulsion.py
# Description : read button state to drive propulsion 
# Author      : Man'O'AR
# modification: 22/04/2023
########################################################################

import odroid_wiringpi as wiringpi
from time import monotonic

class PropulsionButton( object ):
    def __init__(
            self, 
            master, 
            pin = 6,
            backward_threshold = 3, 
            verbose = False ):
        
        super().__init__()

        self._master = master
        self._verbose = verbose

        self._pinout = pin

        self._isPressed = False
        self._isLongPressed = False

        self._backward_threshold = backward_threshold

        self._press_start = 0
        self._press_duration = 0

        self._propulsion_is_on_standby = True

    def start( self ):
        
        wiringpi.wiringPiSetup()
        
        wiringpi.pinMode( self._pinout, wiringpi.INPUT )
        wiringpi.pullUpDnControl( self._pinout, wiringpi.PUD_UP )

    def send_stp( self ):
        
        self._propulsion_is_on_standby = True
        self._isLongPressed = False

        if self._master is not None:
            self._master._update_propulsion( spin_direction = "stp" )


    def send_fwd( self ):
        
        self._propulsion_is_on_standby = False
        self._isLongPressed = False

        if self._master is not None:
            self._master._update_propulsion( spin_direction = "fwd" )


    def send_bwd( self ):

        self._isLongPressed = True
        self._propulsion_is_on_standby = False

        if self._master is not None:
            self._master._update_propulsion( spin_direction = "bwd" )



    def _read_state( self ):

        # Read button state
        state = wiringpi.digitalRead( self._pinout )

        # If button is pressed
        if state == wiringpi.LOW and not self._isPressed:
            self._press_start =  monotonic()
            self._isPressed = True

        # Calculate press duration
        if self._isPressed is True:
            self._press_duration = monotonic() - self._press_start

        # If button is released
        if state == wiringpi.HIGH and self._isPressed:

            if self._press_duration <= self._backward_threshold :

                if self._propulsion_is_on_standby is True:
                    
                    self.send_fwd()

                else:
                    
                    self.send_stp()

            else:

                self.send_stp()
    
            self._isPressed = False

        if self._isPressed and self._press_duration > self._backward_threshold :

            if self._press_duration > self._backward_threshold:

                if self._isLongPressed is False: 
                    self.send_bwd()

    def exit( self ):
        print("propulsion button is disabled")