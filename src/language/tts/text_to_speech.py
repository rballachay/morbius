from config import ACTIVE_TTS, TTS_MODELS
from src.language.tts.fast_speech import FastSpeech
from src.language.tts.styleTTS2 import StyleTTS2
import pyaudio
import numpy as np

class TextToSpeech:
    def __init__(self):
        if ACTIVE_TTS not in TTS_MODELS:
            raise Exception(f"TTS model must be one of: {', '.join(TTS_MODELS)}")
        
        if ACTIVE_TTS=='styleTTS2':
            self.model=StyleTTS2()
        elif ACTIVE_TTS=='fast_speech':
            self.model=FastSpeech()

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
