import os

"""GLOBAL"""
DATA_PATH = "data"
MODEL_PATH = "models"
LOG_PATH = ".logs"
DEVICE='cpu'
"""GLOBAL"""

"""TRANSCRIBE"""
CACHE_RECORDINGS = False
RECORDINGS_DIR = f"{DATA_PATH}/recordings"
"""TRANSCRIBE"""

"""RECORDING"""
RECORD_LENGTH = 15  # the length of the recording to use, ignoring silence
SILENCE_LENGTH = 1  # length of silence before cutting recording
VAD_MODE = 2  # this goes from 0->3, see here: https://github.com/dpirch/libfvad/blob/master/include/fvad.h#L52
"""RECORDING"""

"""WHISPER"""
# have to set this variable when running, as its not set up for mac
os.environ["WHISPER_CPP_LIB"] = (
    "/Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/libwhisper.dylib"
)
WHISPER_DIR = f"{MODEL_PATH}/whisper"
WHISPER_SIZES = ["tiny", "base", "small", "medium"]
WHISPER_LOCAL = "ggml-torch-{model_size}.bin"
WHISPER_URL = (
    "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-{model_size}.en.bin"
)
"""WHISPER"""

"""RASA"""
RASA_MODEL_PATH = "models/rasa-model-june10.tar.gz"
RASA_ACTIONS_PATH = "data.rasa.actions"
RASA_PORT = 5005
ACTIONS_PORT = 5055
"""RASA"""

"""FASTSPEECH"""
PREPROCESS_CONFIG='submodules/FastSpeech2/config/LJSpeech/preprocess.yaml'
MODEL_CONFIG='submodules/FastSpeech2/config/LJSpeech/model.yaml'
HIFIGAN_CONFIG='submodules/FastSpeech2/hifigan/config.json'
HIFIGAN_MODEL='submodules/FastSpeech2/hifigan/generator_LJSpeech.pth.tar'
FAST_MODEL_PATH='models/fastspeech-model.pth.tar'
"""FASTSPEECH"""