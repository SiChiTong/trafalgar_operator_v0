#!/usr/bin/env python
import traceback
import datetime
import rclpy
import cv2 # OpenCV library
import math
from threading import Thread,Lock
import customtkinter
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("dark")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

from ..components.__streamNode import VideoStream
    
class Display(customtkinter.CTk):

    APP_NAME = ""
    WIDTH = 480
    HEIGHT = 480

    def __init__(
            self
        ):
        
        super().__init__()
        
        self.EnableText = False
        self.ZoomLevel = 10

        self.arrowAngle = 90
        self.arrowColor = "white"

        
        self._drone_index = None
        self._node = None

        self.title(Display.APP_NAME)

        self.geometry(str( self.winfo_screenwidth()  ) + "x" + str(self.winfo_screenheight() ))
        self.minsize(Display.WIDTH, Display.HEIGHT)

        self.attributes("-fullscreen", True) 

        self.canvaResolution = (self.winfo_screenwidth(), self.winfo_screenheight()) 

        self.bind("<Control-c>", self._closing_from_gui)

        self._frame = None
        self.last_frame = None
        self._is_frame_updated = False
        
        self._blackScreen = False 
        self._isGamePlayEnable = False

        self._textToDisplay = "GAME OVER"
        self.iddleLoop = False

        self._playtimeLeft = 0
        self._loop_delay = 1

        self._thread = None
        self._lock = Lock()

        self._is_running = True

        self._initialize()

    @property
    def videoWidth(self):
        return 480

    @property
    def videoHeight(self):
        return 320
        
    def _initialize( self ):    

        self._start_rosNode()
        self._create_window()
        self.update()

    
    def _start( self ):
        self.mainloop()
    

    def _create_window( self ): 

        self.canvas = customtkinter.CTkCanvas(self, width=self.canvaResolution[0], height=self.canvaResolution[1])
        self.canvas.pack( side="top", fill="both", expand=True )
        
        self.draw_header_footer()
        self.draw_directional_arrow()
        self.drawBlackScreen()


    def draw_header_footer(self):
        # Header
        self.canvas.create_rectangle(0, 0, 480, 60, fill="black", tags="header")

        # Footer
        self.canvas.create_rectangle(0, 380, 480, 480, fill="black", tags="footer")
        self.canvas.create_text(50, 430, text=self.droneName, fill="white", anchor="w", tags="drone_name")


    def draw_directional_arrow(self):
        cx, cy = 240, 430  # Position adjusted

        points = [
            cx, cy - 15, 
            cx + 12, cy,      
            cx + 3, cy,       
            cx + 3, cy + 8,   
            cx - 3, cy + 8,   
            cx - 3, cy,       
            cx - 12, cy       
        ]

        pivot_x = sum(points[::2]) / len(points[::2])
        pivot_y = sum(points[1::2]) / len(points[1::2])

        angle_rad = math.radians(self.current_angle - 90)

        for i in range(0, len(points), 2):
            x, y = points[i] - pivot_x, points[i+1] - pivot_y
            points[i] = pivot_x + (x * math.cos(angle_rad) - y * math.sin(angle_rad))
            points[i+1] = pivot_y + (x * math.sin(angle_rad) + y * math.cos(angle_rad))

        self.canvas.create_polygon(points, outline=self.arrowColor, fill=self.arrowColor, tags="arrow")



    def get_direction(self):
        if self.current_angle >= 80 and self.current_angle <= 100:
            return f"CENTRE: 0°"
        elif self.current_angle < 80:
            return f"GAUCHE: {self.current_angle - 90}°"
        else:
            return f"DROIT: {self.current_angle - 90}°"
        
        
    def crop_from_center(self, frame, frame_width, frame_height, ZoomLevel ):
        
        ZoomLevel = 100 - ZoomLevel
        
        if ZoomLevel == 0:
            return frame

        elif ZoomLevel == 100:
            ZoomLevel = 90

        crop_width = int(frame_width * ZoomLevel/100)
        crop_height = int(frame_height * ZoomLevel/100)

        start_x = (frame_width - crop_width) // 2
        start_y = (frame_height - crop_height) // 2

        end_x = start_x + crop_width
        end_y = start_y + crop_height
    
        cropped_image = frame[start_y:end_y, start_x:end_x]
    
        return cropped_image


    def _rosVideoUpdate( self, frame, frameWidth, frameHeight, playTime = 10*60 ):

        if( frame is not None and self._isGamePlayEnable is True ):

            cropFrame = self.crop_from_center(frame, frameWidth, frameHeight, self.ZoomLevel )
            resized_frame = cv2.resize(  cropFrame , ( self.videoWidth, self.videoHeight ))
            color_conv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

            img = Image.fromarray(color_conv)
            img_canvas = ImageTk.PhotoImage(img)

            self._frame = img_canvas
            self._is_frame_updated = True
    

    def set_text( self, txt ): 
        self._textToDisplay = txt
    

    def render_text_at_center( self ):
        
        #ideal_font_size = 80

        window_width = self.canvas.winfo_width()
        window_height = self.canvas.winfo_height()
        
        font_size = 40 #min(window_width, window_height) * ideal_font_size // 1080
        
        text = self.canvas.create_text(
            0, 
            0, 
            text=self._textToDisplay, 
            fill="white", 
            font=("Arial", font_size, "bold"), 
            justify=customtkinter.CENTER
        )
        
        text_bbox = self.canvas.bbox(text)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        text_x = (window_width - text_width) // 2 + text_width / 2
        text_y = (window_height - text_height) // 2 + text_height / 2

        self.canvas.coords(text, text_x, text_y)

    def drawBlackScreen( self ):

        if self._blackScreen is False: 

            self.canvas.update_idletasks()

            self.canvas.delete("video")
            self.canvas.delete("arrow")

            self.canvas.create_rectangle(0, 0, self.canvas.winfo_width(), self.canvas.winfo_height(), fill="black")
            
            if self.EnableText is True:
                self.render_text_at_center()

            self._blackScreen = True    
      

    def renderVideoFrame( self ):

        if self._is_frame_updated is True:

            if self.last_frame != self._frame:

                self.last_image = self._frame
                self.canvas.delete("video")
                self.canvas.create_image(0, 0, anchor="nw", image=self.last_image, tags="video")

                self._blackScreen = False

    def renderElapsedTime( self ):

        self._playtimeLeft = self._node._playtimeLeft

        if self._playtimeLeft > 0:

            time_delta = datetime.timedelta(seconds=self._playtimeLeft)
            formatted_time = str(time_delta).split(".")[0]

            self.canvas.delete("timer")

            self.canvas.create_text(
                240, 
                30, 
                text=f"Temps restant: { formatted_time }", 
                fill="white", 
                font=("Arial", 20, "bold"), 
                anchor="w", 
                tags="timer"
            )



    def update(self):
    
        if self._isGamePlayEnable is True:

            self.renderVideoFrame()
            self.renderElapsedTime()
            self.draw_directional_arrow()

        else:

            self.drawBlackScreen()

        self._is_frame_updated = False

        self.after(self._loop_delay, self.update)



    def _stop(self):
        
        #sleep(2)
        if self._node is not None:
            with self._lock:
                try:
                    self._kill_rosNode() 

                    print("ros node has been killed")
                    
                    if self._thread is not None: 
                        self._thread.join()
                    
                except Exception:
                    pass

        self.destroy()


    def _action_on_shutdown(self, value):
        
        self._stop()


    def _closing_from_gui(self, event):

        self._stop()

    

    def _start_rosNode( self):

        self._thread = Thread( target=self._rosNode_thread)
        self._thread.daemon = True
        self._thread.start()

    def _rosNode_thread(self):

        if self._node is None:
            self._node = VideoStream(Master=self) 
        
        self._is_running = True
        
        with self._lock:

            try:

                rclpy.spin( self._node )

            except Exception as exception:
                print( "an exception has been raised while spinning the movement node : ", exception)

            except KeyboardInterrupt:
                print("user force interruption")

        
    def _kill_rosNode(self):

        if self._node is not None:
            self._node.destroy_node()
            rclpy.shutdown()




def main(args=None):

    rclpy.init(args=args)

    app = Display()

    try:

        app._start()

    except Exception as e:
        traceback.print_exc()

    finally:

        app._stop()

        

if __name__ == '__main__':
    main()

    