#!/usr/bin/env python3
########################################################################
# Filename    : __audioManager.py
# Description : audio manager
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################
import os
from pygame import mixer

class AudioManager(object):

    MUSIC_DIR = "media/audio/music"
    SFX_DIR = "media/audio/sfx"

    def __init__(self, args=None ):
        
        super().__init__()

        self._mixer = None
        self._is_running = True
        self._music_playlist = []
        self._sfx_playlist = []

    
    def _enable( self ):
        
        mixer.init()
        self._mixer = mixer.music

        self._load_music()
        self._load_sfx()

    def _load_music( self ):
        
        #print( os.path.join(self.MUSIC_DIR, filename) )

        print('basename:    ', os.path.basename(__file__))
        print('dirname:     ', os.path.dirname(__file__))


        self._music_playlist = [
            os.path.join(self.MUSIC_DIR, filename)
            for filename in os.listdir(self.MUSIC_DIR)
            if filename.endswith(".wav")
        ]

    def _load_sfx( self ):

        self._sfx_playlist = [
            os.path.join(self.SFX_DIR, filename)
            for filename in os.listdir(self.SFX_DIR)
            if filename.endswith(".wav")
        ]


    def play_sfx( self, sfx = None ): 

        if self._mixer and sfx in self._sfx_playlist:
            self._mixer.Sound(sfx).play()


    def play_music( self, music = None ): 

        if self._mixer and music in self._music_playlist:
            if self._mixer.get_busy():
                self._mixer.stop()
            self._mixer.load(music)
            self._mixer.play(loops=-1)

    def stop_music( self ): 

        if self._mixer and self._mixer.get_busy():
            self._mixer.stop()

    def _disable( self ):
        
        if self._mixer is not None:
            self._mixer.music.stop()
            self._mixer.quit()
                 
                 
