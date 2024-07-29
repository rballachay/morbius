from src.stt.record import record_until_thresh
from src.file_utils import download_file
from config import CACHE_RECORDINGS, RECORDINGS_DIR, \
    WHISPER_DIR, WHISPER_LOCAL, WHISPER_SIZES, WHISPER_URL, SAMPLE_RATE, DEVICE
import os
from whisper_cpp_python import Whisper as WhisperCPP
from faster_whisper import WhisperModel
import time
from scipy.io.wavfile import write
import numpy as np

class FasterWhisper:
    """Faster version of the whisper class below, also uses int8 
    precision. Takes raw audio data at a sampling rate of 
    16000. Outputs the  
    """
    def __init__(self, model_size:str):
        model_size = f"{model_size}.en"

        # Run on CPU with FP32
        self.whisper = WhisperModel(model_size, device=DEVICE, compute_type="int8")

    def transcribe(self, data):
        segments,_ = self.whisper.transcribe(data)
        return ''.join([segment.text for segment in segments])

class Whisper:
    """Whisper cpp, voice-to-text model. Not quite fast enough for production. 
    """
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

    def transcribe(self, data):
        return self.whisper._full(data)["text"]

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

    if CACHE_RECORDINGS:
        filename = f"{RECORDINGS_DIR}/morbius_recording_{int(time.time())}.wav"
        _create_subfolders_for_file(filename)

    try:
        data = record_until_thresh()
    except Exception as e:
        print("Failed to transcribe audio, check shared library record_audio.so")
        return ""

    results = whisper.transcribe(data)

    if CACHE_RECORDINGS:
        scaled = np.int16(data / np.max(np.abs(data)) * 32767)
        write(filename, SAMPLE_RATE, scaled)

    return results

def _create_subfolders_for_file(file_path):
    directory = os.path.dirname(file_path)

    if directory and not os.path.exists(directory):
        os.makedirs(directory)
        print(f"Directories created for {directory}")
    else:
        print(f"Directory {directory} already exists")
