from config import ACTIVE_TTS, TTS_MODELS
from src.language.tts.fast_speech import FastSpeech
from src.language.tts.styleTTS2 import StyleTTS2
from src.language.tts.espeak import ESpeak
from src.language.tts.nix_tts import NixTTS
import pyaudio
import numpy as np

class TextToSpeech:
    def __init__(self, active_tts=ACTIVE_TTS, tts_models=TTS_MODELS):
        if active_tts not in tts_models:
            raise Exception(f"TTS model must be one of: {', '.join(tts_models)}")
        
        if active_tts=='styleTTS2':
            self.model=StyleTTS2()
        elif active_tts=='fast_speech':
            self.model=FastSpeech()
        elif active_tts=='espeak':
            self.model=ESpeak()
        elif active_tts=='nix_tts':
            self.model = NixTTS()

        self.pyaudio = pyaudio.PyAudio()
        
    def speak(self, message):

        wav_predictions = self.model(message)

        stream = self.pyaudio.open(format=pyaudio.paFloat32,
                         channels=1,
                         rate=self.model.sampling_rate,
                         output=True,
                         output_device_index=1
                         )
                
        stream.write(wav_predictions.astype(np.float32).tostring())
