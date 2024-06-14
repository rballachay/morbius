# add the submodules/FastSpeech2 path so that we 
# can import model and hifigan normally from here
import sys
sys.path.append('submodules/nix-tts')

from nix.models.TTS import NixTTSInference
from config import NIX_TTS_MODEL

class NixTTS:
    def __init__(self):
        self.sampling_rate=22050
        # Initiate Nix-TTS
        self.nix = NixTTSInference(model_dir = NIX_TTS_MODEL)

    def __call__(self, text):
        c, c_length, phoneme = self.nix.tokenize(text)
        # Convert text to raw speech
        xw = self.nix.vocalize(c, c_length)

        return xw[0,0,:]