import pvporcupine
import pyaudio
import struct
from config import PICO_ACCESS_KEY, PORCUPINE_MODEL_MAC, PORCUPINE_MODEL_PI
import platform
from controller import VoiceController, RasaManager


def controller_handler(rasa):
    vc = None
    vc = VoiceController(daemon=True, rasa=rasa)
    vc.run()


def get_porcupine_model():
    system = platform.system()
    machine = platform.machine()

    if system == "Darwin":
        return PORCUPINE_MODEL_MAC
    elif system == "Linux":
        if machine in ["armv7l", "armv6l", "aarch64"]:
            return PORCUPINE_MODEL_PI
    raise Exception("Cannot load daemon on non mac/raspberry pi")


# Function to trigger the full process - this can be blocking
def trigger_full_process(rasa):
    print("Hey Robot detected! Triggering full process...")
    # Add code here to start the full process, such as calling another script or service
    controller_handler(rasa)


def create_audio_stream(porcupine):
    pa = pyaudio.PyAudio()

    audio_stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=porcupine.frame_length,
    )

    return pa, audio_stream


def close_audio_stream(pa, audio_stream):
    if not audio_stream is None:
        audio_stream.close()
    if not pa is None:
        pa.terminate()
    return None, None


# Function to run the daemon process
def listen_for_keyword():
    rasa = RasaManager()
    keyword_path = (
        get_porcupine_model()
    )  # Download the "hey robot" ppn file from Picovoice Console

    porcupine = pvporcupine.create(
        access_key=PICO_ACCESS_KEY, keyword_paths=[keyword_path]
    )

    pa = None
    audio_stream = None

    print("Listening for 'Hey Robot'...")

    try:
        while True:
            if pa is None:
                pa, audio_stream = create_audio_stream(porcupine)

            pcm = audio_stream.read(porcupine.frame_length)
            pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

            keyword_index = porcupine.process(pcm)
            if keyword_index >= 0:
                pa, audio_stream = close_audio_stream(pa, audio_stream)
                trigger_full_process(rasa)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        close_audio_stream(pa, audio_stream)
        porcupine.delete()
        rasa.rasa_controller.terminate()
        rasa.action_controller.terminate()


if __name__ == "__main__":
    listen_for_keyword()
