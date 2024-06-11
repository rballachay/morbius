from src.language.record import record_until_thresh
from src.file_utils import download_file
from config import CACHE_RECORDINGS, RECORDINGS_DIR, \
    WHISPER_DIR, WHISPER_LOCAL, WHISPER_SIZES, WHISPER_URL
import os
from whisper_cpp_python import Whisper as WhisperCPP

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
        return self.whisper._full(filename)["text"]

def download_whisper(model_size):
    """Download whisper model to directory indicated in config"""
    if not model_size in WHISPER_SIZES:
        raise Exception(f"Whisper size must be one of {', '.join(WHISPER_SIZES)}")

    url = WHISPER_URL.format(model_size=model_size)
    filename = f"{WHISPER_DIR}/{WHISPER_LOCAL.format(model_size=model_size)}"

    # return true/false based on successful download (false if already downloaded)
    return download_file(url, filename)


def listen_transcribe(whisper: Whisper):
    """This listens to audio until silence, then transcribes audio to text
    using whisper.
    """

    #if CACHE_RECORDINGS:
    #    filename = f"{RECORDINGS_DIR}/morbius_recording_{int(time.time())}.wav"
    #else:
    #    filename = f"{RECORDINGS_DIR}/_temp_audio.wav"

    try:
        data = record_until_thresh()
    except Exception as e:
        raise Exception(e)
        print("Failed to transcribe audio, check shared library record_audio.so")
        return ""

    results = transcribe_audio(data, whisper)

    #if not CACHE_RECORDINGS:
    #    os.remove(filename)

    return results


def transcribe_audio(data, whisper):
    """Transcribe the audio file using Whisper."""
    return whisper.transcribe(data)
