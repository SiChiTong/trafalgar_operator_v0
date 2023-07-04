#!/usr/bin/env python
import cv2 # OpenCV library

import customtkinter
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("dark")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

    
class Display(customtkinter.CTk):

    APP_NAME = ""
    WIDTH = 480
    HEIGHT = 480

    def __init__(
            self, 
            Master=None
        ):
        
        super().__init__()

        self._master = Master
        
        self._navigation_marker = None

        self._drone_index = None

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
        
        self._timeLeft = 0
        self._loop_delay = 1

        self._initialize()
        
    def _initialize( self ):

        self._create_window()
        self._render_frame()
        #self._loop()


    def _start( self ):

        self.mainloop()


    def _loop( self ):
        
        if self._isGamePlayEnable is True:
            print("elapsedTime")

        self.after(1000, self._loop)#wait for 1 second

    def _create_window( self ): 

        self.canvas = customtkinter.CTkCanvas(self, width=self.canvaResolution[0], height=self.canvaResolution[1])
        self.canvas.pack( side="top", fill="both", expand=True )
        
        self.drawBlackScreen()

    def _rosVideoUpdate( self, frame, gameplay_enable = False, playTime = 10*60 ):
        
        self._isGamePlayEnable = gameplay_enable

        if( frame is not None and self._isGamePlayEnable ):
            
            resized_frame = cv2.resize( frame, ( self.canvas.winfo_width(), self.canvas.winfo_height() ))
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

            self.canvas.delete("all")

            self.canvas.create_rectangle(0, 0, self.canvas.winfo_width(), self.canvas.winfo_height(), fill="black")
            self.render_text_at_center()

            self._blackScreen = True    
      

    def drawVideoFrame( self ):

        if self._is_frame_updated is True:

            if self.last_frame != self._frame:

                self.last_image = self._frame
                self.canvas.delete("all")
                self.canvas.create_image(0, 0, anchor="nw", image=self.last_image)

                self._blackScreen = False


    def _render_frame(self):
    
        if self._isGamePlayEnable is True:
            self.drawVideoFrame()

        else: 
            self.drawBlackScreen()

        self._is_frame_updated = False

        self.after(self._loop_delay, self._render_frame)



    def _stop(self):
        
        #sleep(2)
        self.destroy()


    def _action_on_shutdown(self, value):
        
        self._stop()


    def _closing_from_gui(self, event):
        
        self._stop()

    

def main(args=None):

    app =  Display()

    try:

        app._start()

    except Exception as exception:
        print( "an exception has been raised while spinning the movement node : ", exception)

    finally:

        app._stop()

        

if __name__ == '__main__':
    main()

    