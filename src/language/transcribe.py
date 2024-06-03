from src.language.record import record_until_thresh
from src.language.whisper import Whisper
from config import CACHE_RECORDINGS, RECORDINGS_DIR
import time


async def listen_transcribe(model_size="tiny"):
    """This listens to audio until silence, then transcribes audio to text
    using whisper.
    """

    if CACHE_RECORDINGS:
        filename = f"{RECORDINGS_DIR}/morbius_recording_{int(time.time())}.wav"
    else:
        filename = f"{RECORDINGS_DIR}/_temp_audio.wav"

    recording_successful = await record_until_thresh(filename)
    if not recording_successful:
        print("Failed to transcribe audio, check shared library record_audio.so")
        return ""

    return await transcribe_audio(filename, model_size)


async def load_model_async(model_size):
    whisper = Whisper(model_size)
    return whisper


async def transcribe_audio(filename, model_size="tiny"):
    """Transcribe the audio file using Whisper."""
    whisper = await load_model_async(model_size)
    text = whisper.transcribe(filename)
    return text
