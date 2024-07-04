# add the current path so that we 
# can import model and nix normally from here
import sys
sys.path.append("src/language/tts")

from nix.models.TTS import NixTTSInference
from config import NIX_TTS_MODEL, NIX_GDRIVE_LINK
from src.file_utils import download_model_gdrive
import os


class NixTTS:
    def __init__(self):
        self.sampling_rate=22050

        # download the model from gdrive if it doesnt exist locally
        if not os.path.exists(NIX_TTS_MODEL):
            download_model_gdrive(NIX_GDRIVE_LINK, NIX_TTS_MODEL, is_zip=True)

        # Initiate Nix-TTS
        self.nix = NixTTSInference(model_dir = NIX_TTS_MODEL)

    def __call__(self, text):
        c, c_length, phoneme = self.nix.tokenize(text)
        # Convert text to raw speech
        xw = self.nix.vocalize(c, c_length)

        return xw[0,0,:]