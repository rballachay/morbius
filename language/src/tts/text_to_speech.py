from config import ACTIVE_TTS, TTS_MODELS
import pyaudio
import numpy as np
import sounddevice as sd
import subprocess
import platform

def is_raspberry_pi():
    pi_models = ['armv6l', 'armv7l', 'aarch64']
    return platform.machine().lower() in pi_models

class TextToSpeech:
    """Class for loading a different text to speech model baed on the config 
    and then speaking the text into audio using pyaudio.

    This will check if the script is being run on a raspberry pi, and 
    if it is, will force using aplay, as pyaudio experiences problems
    on the raspberry pi for some reason.
    """
    def __init__(self, active_tts=ACTIVE_TTS, tts_models=TTS_MODELS):

        if active_tts not in tts_models:
            raise Exception(f"TTS model must be one of: {', '.join(tts_models)}")
        
        # lazy load the tts model
        if active_tts=='styleTTS2':
            from src.tts.experimental.styleTTS2 import StyleTTS2
            self.model=StyleTTS2()
        elif active_tts=='fast_speech':
            from src.tts.experimental.fast_speech import FastSpeech
            self.model=FastSpeech()
        elif active_tts=='espeak':
            from src.tts.experimental.espeak import ESpeak
            self.model=ESpeak()
        elif active_tts=='nix_tts':
            from src.tts.nix_tts import NixTTS
            self.model = NixTTS()

        self.is_pi =  is_raspberry_pi()
        self.devices = sd.query_devices()

        if self.is_pi:
            self.pyaudio=None
        else:
            self.pyaudio = pyaudio.PyAudio()
        
    def speak(self, message):

        wav_predictions = self.model(message)

        if self.is_pi:
           self.__stream_pi(wav_predictions)
        else:
            self.__stream(wav_predictions)

    def __stream(self, wav_predictions):
        """Default form of streaming audio, uses pyaudio
        """
        stream = self.pyaudio.open(format=pyaudio.paFloat32,
                         channels=self.channels,
                         rate=self.sampling_rate,
                         output=True,
                         output_device_index=self.device_index
                         )
                
        stream.write(wav_predictions.astype(np.float32).tostring())

    def __stream_pi(self, wav_predictions):
        """pyaudio refuses to cooperate on the raspberry pi, so instead
        of losing time trying to make it work, just use aplay in bash.
        """
        audio_data_int16 = np.int16(wav_predictions * 32767)
        audio_data_bytes = audio_data_int16.tobytes()
        command = ['aplay', '-f', 'S16_LE', '-r', f'{int(self.sampling_rate/2)}', '-c', '2', '-']

        with subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE) as proc:
            # Write the converted audio data to stdin
            proc.stdin.write(audio_data_bytes)
            proc.stdin.close()
            proc.wait()

    @property 
    def channels(self):
        selected_device = self.devices[self.device_index]
        if selected_device['name']=='Jabra SPEAK 410 USB':
            return 2
        return 1
    
    @property 
    def device_index(self):
        if self.devices[0]['name']=='Jabra SPEAK 410 USB':
            return 0
        return 1
    
    @property 
    def sampling_rate(self):
        if (self.devices[self.device_index]['name']=='Jabra SPEAK 410 USB') and not self.is_pi:
            return int(self.model.sampling_rate/2)
        else:
            return self.model.sampling_rate