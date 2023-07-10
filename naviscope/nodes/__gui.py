#!/usr/bin/env python3
########################################################################
# Filename    : __gui.py
# Description : main app for gcs
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

from threading import Thread,Lock

import rclpy

from ..components.__userInterfaceOp import Display
from ..components.__streamNode import VideoStream

class App:

    def __init__(self, args=None ):

        self._gui = None
        self._node = None

        self._thread = None
        self._lock = Lock()

        self._is_running = True

        rclpy.init(args=args)
        self._node = VideoStream(Master=self) 

    def run(self):

        self._gui = Display( Master=self )

        self._thread = Thread(target=self.ros_thread)
        self._thread.daemon=True
        self._thread.start()
        
        self._gui._start( )


    def ros_thread(self):

        try:

            rclpy.spin( self._node )

        except Exception as exception:
            print( "an exception has been raised while spinning the movement node : ", exception)

        except KeyboardInterrupt:
            print("user force interruption")
        
        
    def kill_ros(self):

        self._node.destroy_node()
        rclpy.shutdown()


    def shutdown( self ):
        
        if self._node is not None:

            try:
                self.kill_ros() 
                
                if self._thread is not None: 
                    self._thread.join()
            
            except Exception:
                pass
                 
                 
def main(args=None):

    app = App(args)

    try :

        app.run()
        app.shutdown()

    except Exception as ex: 
        print( ex )



if __name__ == '__main__':
    main()