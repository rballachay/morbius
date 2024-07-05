from config import ACTIVE_TTS, TTS_MODELS
import pyaudio
import numpy as np

class TextToSpeech:
    def __init__(self, active_tts=ACTIVE_TTS, tts_models=TTS_MODELS):
        if active_tts not in tts_models:
            raise Exception(f"TTS model must be one of: {', '.join(tts_models)}")
        
        # lazy load the tts model
        if active_tts=='styleTTS2':
            from src.language.tts.styleTTS2 import StyleTTS2
            self.model=StyleTTS2()
        elif active_tts=='fast_speech':
            from src.language.tts.fast_speech import FastSpeech
            self.model=FastSpeech()
        elif active_tts=='espeak':
            from src.language.tts.espeak import ESpeak
            self.model=ESpeak()
        elif active_tts=='nix_tts':
            from src.language.tts.nix_tts import NixTTS
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
