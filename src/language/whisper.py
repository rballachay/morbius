from config import WHISPER_DIR, WHISPER_LOCAL, WHISPER_SIZES, WHISPER_URL
from src.utils import download_file
import os
from whisper_cpp_python import Whisper as WhisperCPP
from scipy.io import wavfile


class Whisper:
    def __init__(self, model_size: str):
        if not model_size in WHISPER_SIZES:
            raise Exception(f"Whisper size must be one of {', '.join(WHISPER_SIZES)}")

        self.model_path = f"{WHISPER_DIR}/{WHISPER_LOCAL.format(model_size=model_size)}"

        if not os.path.exists(self.model_path):
            print(
                f"Model size {model_size} is unavailable, downloading from huggingface..."
            )
            if not download_whisper(model_size):
                raise Exception("Failed to download whisper :(")

        self.whisper = WhisperCPP(model_path=self.model_path)

    def transcribe(self, filename):
        return self.whisper.transcribe(filename)["text"]


def download_whisper(model_size):
    """Download whisper model to directory indicated in config"""
    if not model_size in WHISPER_SIZES:
        raise Exception(f"Whisper size must be one of {', '.join(WHISPER_SIZES)}")

    url = WHISPER_URL.format(model_size=model_size)
    filename = f"{WHISPER_DIR}/{WHISPER_LOCAL.format(model_size=model_size)}"

    # return true/false based on successful download (false if already downloaded)
    return download_file(url, filename)
