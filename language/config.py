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

"""SILENCE"""
# silent, for development, allows us to write in verbal commands via
# the shell instead of saying them, to make it easier
SILENT_IN = False
SILENT_OUT = False
SILENT_ROBOT = False
"""SILENCE"""

"""GLOBAL"""
DATA_PATH = "data"
MODEL_PATH = "models"
LOG_PATH = ".logs"
DEVICE='cpu'
ROBOT_ID=1
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
RASA_MODEL_PATHS = {"v1":"models/rasa-model-june10.tar.gz", "v2":"models/rasa-model-july22.tar.gz"}
RASA_ACTIONS_PATHS = {"v1":"data.rasa.v1_full.actions","v2":"data.rasa.v2_motor.actions"}
RASA_MODELS_GDRIVE={
    "v1":"https://drive.google.com/file/d/11AfeXgzIohEwyRLv8DaGWP9Q2gNtd0zY/view?usp=sharing",
    "v2":"https://drive.google.com/file/d/1MepvLIZ-pqB5497DTWpzwnahzWKJCB-R/view?usp=sharing"
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
elif os.path.exists('/opt/homebrew/lib/libespeak.dylib'):
    PHONEMIZER_ESPEAK_LIBRARY = '/opt/homebrew/lib/libespeak.dylib'
else:
    PHONEMIZER_ESPEAK_LIBRARY = f"{find_library_path('espeak')}/libespeak-ng.dylib"

if PHONEMIZER_ESPEAK_LIBRARY is None:
    raise Exception("Ensure espeak is installed and add path to `libespeak.{dylib/so}` here")

os.environ["PHONEMIZER_ESPEAK_LIBRARY"] = PHONEMIZER_ESPEAK_LIBRARY
"""TTS CONFIG"""

####################### EXPERIMENTAL #######################
# These are experimental and can be activated by adding the submodules and necessary 
# models to these paths.
"""FASTSPEECH"""
# find at https://github.com/ming024/FastSpeech2
# models at https://drive.google.com/drive/folders/1DOhZGlTLMbbAAFZmZGDdc77kz1PloS7F
PREPROCESS_CONFIG='submodules/FastSpeech2/config/LJSpeech/preprocess.yaml'
MODEL_CONFIG='submodules/FastSpeech2/config/LJSpeech/model.yaml'
HIFIGAN_CONFIG='submodules/FastSpeech2/hifigan/config.json'
HIFIGAN_MODEL='submodules/FastSpeech2/hifigan/generator_LJSpeech.pth.tar'
FAST_MODEL_PATH='models/fastspeech-model.pth.tar'
"""FASTSPEECH"""

"""STYLETTS2"""
# find at https://github.com/yl4579/StyleTTS2
# models at https://huggingface.co/yl4579/StyleTTS2-LJSpeech/tree/main
LJ_CONFIG="models/LJSpeech/config.yml"
LJ_MODEL="models/LJSpeech/epoch_2nd_00100.pth"
"""STYLETTS2"""
####################### EXPERIMENTAL #######################

"""NIX_TTS"""
NIX_TTS_MODEL="models/nix-ljspeech-deterministic-v0.1"
NIX_GDRIVE_LINK="https://drive.google.com/file/d/1dN__W2TUiJ4hgH2ulARnBGyNPA_40OhQ/view?usp=sharing"
"""NIX_TTS"""

"""ROBOT_DEFAULT"""
DEFAULT_ANGLE=45 #degrees
DEFAULT_FORWARD=50 #cm
DEFAULT_BACKWARD=50 #cm
"""ROBOT_DEFAULT"""

"""VOICE KEYWORD RECOGNITION"""
PICO_ACCESS_KEY='HXZKfDFpC/RpSZhOkp7m6TfDWDOG63GilPtIhbKc6ARtqP6pKZT9Rw=='
PORCUPINE_MODEL_MAC='models/Porcupine/Hey-Robot_en_mac_v3_0_0.ppn'
PORCUPINE_MODEL_PI='models/Porcupine/Hey-Robot_en_raspberry-pi_v3_0_0.ppn'
"""VOICE KEYWORD RECOGNITION"""

"""VISION MODEL PORT"""
VISION_MODEL_PORT=9080 # this value is also in 'vision/config.json. if you are
                       # updating it there, you need to update it here
"""VISION MODEL PORT"""