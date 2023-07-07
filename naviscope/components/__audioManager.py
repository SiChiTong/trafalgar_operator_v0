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

    def __init__(self, args=None ):
        
        super.__init__()

        self._mixer = None
        
        self._sfx = {
            "track" : "",
            "player" : None,
            "isPlaying" : False
        }

        self._music = {
            "track" : "",
            "player" : None,
            "isPlaying" : False
        }

        self._is_running = True

        self._playlist = {
            "music" :
            {
            "adventure" :  "adventure.wav",
            "combat" :  "combat.wav"
            },
            "sfx":{
            "bell" :  "bell.wav",
            "wheel" :  "wheel.wav",
            } 
        }
 
    def _enable( self ):
        
        mixer.init()
        self._mixer = mixer.music

        self.load_music( )

        # Obtenir le répertoire d'exécution du code
        self._execution_dir = os.path.dirname(os.path.abspath(__file__))

        wav_file_path = os.path.join(
            os.path.dirname(__file__),  # Répertoire actuel du fichier
            'media',                    # Répertoire "media"
            'bell.wav'  # Nom de votre fichier WAV
        )

        print(wav_file_path )
        # Construire le chemin vers le répertoire contenant les fichiers audio
        self._playlist_dir = os.path.join(self._execution_dir, '../media/audio')


    def load_music( self ):

        for playsong in self._playlist["music"].values():
            self._mixer.load( os.path.join(self._playlist_dir, playsong ) )


    def load_sfx( self ):

        for playsong in self._playlist["sfx"].values():
            self._mixer.load( os.path.join(self._playlist_dir, playsong ) )


    def play_sfx( self, sfx = None ): 

        if self._mixer is not None and sfx is not None : 

            if sfx != self._sfx["track"]:
                
                if sfx in self._playlist["sfx"].keys():
                    self._sfx["track"] = self._playlist["sfx"][sfx]
                    self._sfx["player"] = self._mixer.Sound(  self._sfx["track"] )     

            self._sfx["player"].play()


    def play_music( self, music = None ): 

        if self._mixer is not None and music is not None : 
            
            if self._music["isPlaying"] is True:
                self._music["player"].stop()
                self._music["isPlaying"] = False

            if music != self._music["track"]:
                
                if music in self._playlist["music"].keys():
                    self._music["track"] = self._playlist["music"][music]
                    self._music["player"] = self._mixer.music.load(  self._music["track"] )     

            self._music["player"].play()
            self._music["isPlaying"] = True


    def _disable( self ):
        
        if self._mixer is not None:
            self._mixer.music.stop()
            self._mixer.quit()
                 
                 
