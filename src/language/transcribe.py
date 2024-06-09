from src.language.record import record_until_thresh
from src.language.whisper import Whisper
from config import CACHE_RECORDINGS, RECORDINGS_DIR
import time
import os

def listen_transcribe(whisper:Whisper):
    """This listens to audio until silence, then transcribes audio to text
    using whisper.
    """

    if CACHE_RECORDINGS:
        filename = f"{RECORDINGS_DIR}/morbius_recording_{int(time.time())}.wav"
    else:
        filename = f"{RECORDINGS_DIR}/_temp_audio.wav"

    recording_successful = record_until_thresh(filename)
    if not recording_successful:
        print("Failed to transcribe audio, check shared library record_audio.so")
        return ""

    results = transcribe_audio(filename, whisper)

    if not CACHE_RECORDINGS:
        os.remove(filename)

    return results

def transcribe_audio(filename, whisper):
    """Transcribe the audio file using Whisper."""
    return whisper.transcribe(filename)