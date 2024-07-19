import os
import ctypes.util
import subprocess

def find_library_path(library_name):
    try:
        # Use the 'which' command to find the full path of the library
        result = subprocess.run(['which', library_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode == 0:
            library_path = result.stdout.strip()

            # this returns the binary, we want the library
            return os.path.abspath(os.path.join(library_path, "../../lib"))
        else:
            return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None
    
"""GLOBAL"""
DATA_PATH = "data"
MODEL_PATH = "models"
LOG_PATH = ".logs"
DEVICE='cpu'
ROBOT_ID=0
"""GLOBAL"""

"""TRANSCRIBE"""
CACHE_RECORDINGS = False
RECORDINGS_DIR = f"{DATA_PATH}/recordings"
"""TRANSCRIBE"""

"""RECORDING"""
SAMPLE_RATE=16000
RECORD_LENGTH = 15  # the length of the recording to use, ignoring silence
SILENCE_LENGTH = 1  # length of silence before cutting recording
VAD_MODE = 3  # this goes from 0->3, see here: https://github.com/dpirch/libfvad/blob/master/include/fvad.h#L52
"""RECORDING"""

"""WHISPER"""

WHISPER_CPP_LIB = ctypes.util.find_library('whisper')

if WHISPER_CPP_LIB is None:
    raise Exception("whisper-cpp library not found, ensure whisper-cpp-python is installed and path to libwhisper.{dylib/so} is set")

os.environ["WHISPER_CPP_LIB"] = WHISPER_CPP_LIB

WHISPER_SIZE = 'tiny'
WHISPER_DIR = f"{MODEL_PATH}/whisper"
WHISPER_SIZES = ["tiny", "small"]
WHISPER_LOCAL = "ggml-torch-{model_size}.bin"
WHISPER_URL = (
    "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-{model_size}.en.bin"
)
"""WHISPER"""

"""RASA"""
RASA_VERSION='v2'
RASA_MODEL_PATHS = {"v1":"models/rasa-model-june10.tar.gz", "v2":"models/rasa-model-july3.tar.gz"}
RASA_ACTIONS_PATHS = {"v1":"data.rasa.v1_full.actions","v2":"data.rasa.v2_motor.actions"}
RASA_MODELS_GDRIVE={
    "v1":"https://drive.google.com/file/d/11AfeXgzIohEwyRLv8DaGWP9Q2gNtd0zY/view?usp=sharing",
    "v2":"https://drive.google.com/file/d/1qIzFCMuPJkh3HvjWjXUDndtr8DOVADjt/view?usp=sharing"
}
RASA_PORT = 5005
ACTIONS_PORT = 5055
"""RASA"""

# this is the active model for text-to-speech.
"""TTS CONFIG"""
ACTIVE_TTS='nix_tts'  # any of TTS_MODELS
TTS_MODELS=['styleTTS2','fast_speech','espeak','nix_tts']

if os.path.exists('/usr/lib/aarch64-linux-gnu/libespeak.so.1'):
    PHONEMIZER_ESPEAK_LIBRARY = '/usr/lib/aarch64-linux-gnu/libespeak.so.1'
else:
    PHONEMIZER_ESPEAK_LIBRARY = f"{find_library_path('espeak')}/libespeak-ng.dylib"

if PHONEMIZER_ESPEAK_LIBRARY is None:
    raise Exception("Ensure espeak is installed and add path to `libespeak.{dylib/so}` here")

os.environ["PHONEMIZER_ESPEAK_LIBRARY"] = PHONEMIZER_ESPEAK_LIBRARY
"""TTS CONFIG"""

"""FASTSPEECH"""
PREPROCESS_CONFIG='submodules/FastSpeech2/config/LJSpeech/preprocess.yaml'
MODEL_CONFIG='submodules/FastSpeech2/config/LJSpeech/model.yaml'
HIFIGAN_CONFIG='submodules/FastSpeech2/hifigan/config.json'
HIFIGAN_MODEL='submodules/FastSpeech2/hifigan/generator_LJSpeech.pth.tar'
FAST_MODEL_PATH='models/fastspeech-model.pth.tar'
"""FASTSPEECH"""

"""STYLETTS2"""
LJ_CONFIG="models/LJSpeech/config.yml"
LJ_MODEL="models/LJSpeech/epoch_2nd_00100.pth"
"""STYLETTS2"""

"""NIX_TTS"""
NIX_TTS_MODEL="models/nix-ljspeech-deterministic-v0.1"
NIX_GDRIVE_LINK="https://drive.google.com/file/d/1dN__W2TUiJ4hgH2ulARnBGyNPA_40OhQ/view?usp=sharing"
"""NIX_TTS"""