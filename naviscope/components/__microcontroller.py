#!/usr/bin/env python3
########################################################################
# Filename    : __microcontroller.py
# Description : bridge to ESP32 via ArduinoJSON
# Author      : Man'O'AR
# modification: 14/08/2021
########################################################################

import logging
from threading import Thread, Lock
import json
import serial
import serial.tools.list_ports

class externalBoard( object ):

        def __init__( self, callback ):
             
            super().__init__()

            self._callback = callback

            self.PORTS = None
            self.COM_PORT = None
            self.SERIAL = None

            self._thread_ = None
            self._thread_stopped = False
            self._lock = Lock()


        @property
        def BaudRate( self ):
            return 115200 # set Baud rate to 9600
        
        @property
        def Bytesize( self ):
            return 8 # Number of data bits = 8
        
        @property
        def Parity( self ):
            return "N" # No parity //https://www.ibm.com/docs/en/aix/7.1?topic=parameters-parity-bits

        @property
        def StopBits( self ):
            return 1 # Number of Stop bits = 1 

        def _thread_init(self):

            self._thread_ = Thread( target = self._listen_for_outputs )
            self._thread_.daemon = True
            self._thread_.start()

            logging.info( f"serial thread capture has started" )


        def stop(self):
            self._thread_stopped = True

        def _enable( self ):

            self.PORTS = list(serial.tools.list_ports.comports())
            
            if( len(self.PORTS ) > 0 ):

                self.COM_PORT = self.PORTS[0].device

                self.SERIAL = serial.Serial(self.COM_PORT)
                self.SERIAL.baudrate = self.BaudRate 
                self.SERIAL.bytesize = self.Bytesize     
                self.SERIAL.parity = self.Parity    
                self.SERIAL.stopbits = self.StopBits

                self._thread_init()
                print(f" port open on {self.COM_PORT }")


        def _serialize( self, instruction = "direction", data = 0 ):
            
            instructions = {
                    "comp" : f"{instruction}",
                    "data" : data
                }
            
            encodeInstructions = json.dumps( instructions  ).encode("utf-8")
            #print( encodeInstructions )

            return encodeInstructions


        def _dispatch_msg( self, component = None, data = 0 ):

            if( component is not None ):
                self._instruct( self._serialize( component, data ) )


        def _instruct(self, serializeInstruction ):

            self.SERIAL.write(serializeInstruction)
            self.SERIAL.flush()
            #print( serializeInstruction )


        def _listen_for_outputs(self):
            
            if self._thread_stopped is False: 
                
                with self._lock:

                    if( self.SERIAL is not None ):

                        try:
                        
                            line = self.SERIAL.readline() 
                            self._callback( line )

                        except Exception as e: #(ValueError, serial.SerialException)
                            pass
                

        def _disable( self ):

            if( self.SERIAL is not None ):

                self.SERIAL.close()  
                
                self.PORTS = None
                self.COM_PORT = None
                self.SERIAL = None  

            self._thread_stopped  = True
        
            if self._thread_ is not None:

                self._thread_.join()
                self._thread_ = None


            logging.info( f"serial thread has been deactivated" )


