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
        
        super().__init__()

        self._mixer = None
        self.IsAdventurePlaying = False
        self.IsIdlePlaying = False

        self.music_files = []
        self._music_playlist = []

        self.sfx_files = []
        self._sfx_playlist = []

    
    def _enable( self ):
        
        mixer.init()
        self._mixer = mixer.music

        #print('basename:    ', os.path.basename(__file__))
        #print('dirname:     ', os.path.dirname(__file__))

        self.MUSIC_DIR = os.path.join(
        os.getcwd(),  # Répertoire actuel du fichier
        'install',                 # Répertoire 'install' généré par colcon build
        'naviscope',    # Nom de votre package
        'share',                   # Répertoire 'share' dans le package installé
        'naviscope',    # Répertoire de votre package dans 'share'
        'media',                   # Répertoire 'media' dans votre package
        'audio',                   # Répertoire 'audio' dans le répertoire 'media'
        'music'                    # Répertoire 'music' dans le répertoire 'audio'
        )

        self.SFX_DIR = os.path.join(
        os.getcwd(),  # Répertoire actuel du fichier
        'install',                 # Répertoire 'install' généré par colcon build
        'naviscope',    # Nom de votre package
        'share',                   # Répertoire 'share' dans le package installé
        'naviscope',    # Répertoire de votre package dans 'share'
        'media',                   # Répertoire 'media' dans votre package
        'audio',                   # Répertoire 'audio' dans le répertoire 'media'
        'sfx'                    # Répertoire 'music' dans le répertoire 'audio'
        )

        self._load_music()
        self._load_sfx()
     

    def _load_music( self ):
        
        music_files = [
            os.path.join(self.MUSIC_DIR, filename)
            for filename in os.listdir(self.MUSIC_DIR)
            if filename.endswith(".wav")
        ]
        
        self._music_playlist = {os.path.splitext(os.path.basename(file))[0]: file for file in music_files}


    def _load_sfx( self ):

        sfx_files = [
            os.path.join(self.SFX_DIR, filename)
            for filename in os.listdir(self.SFX_DIR)
            if filename.endswith(".wav")
        ]

        self._sfx_playlist = {os.path.splitext(os.path.basename(file))[0]: file for file in sfx_files}


    def play_sfx( self, sfx = None ): 
        
        if self._mixer and sfx in self._sfx_playlist:
            
            sfx_path = self._sfx_playlist[sfx]
            sfxClip = mixer.Sound( sfx_path )
            sfxClip.play()


    def play_music( self, music = None ): 

        if self._mixer and music in self._music_playlist:
            if self._mixer.get_busy():
                self._mixer.stop()
            
            musicToPlay = self._music_playlist[music]
            self._mixer.load(musicToPlay)
            self._mixer.play(loops=-1)


    def gameplayMusic( self, isEnable, direction ): 

        if isEnable is True:

            if direction != 0:

                if self.IsIdlePlaying is True:
                    self.stop_music()
                    self.IsIdlePlaying = False

                if self.IsAdventurePlaying is False:
                    self.IsAdventurePlaying = True
                    self.play_music("adventure")

            else:

                if self.IsAdventurePlaying is True:
                    self.stop_music()
                    self.IsAdventurePlaying = False

                if self.IsIdlePlaying is False:
                    self.IsIdlePlaying = True
                    self.play_music("idle")
                    
        else: 
            
            self.IsAdventurePlaying = False
            self.IsIdlePlaying = False

            self.stop_music()



    def stop_music( self ): 

        if self._mixer and self._mixer.get_busy():
            self._mixer.stop()

    def _disable( self ):
        
        if self._mixer is not None:
            self._mixer.music.stop()
            self._mixer.quit()
                 
                 
