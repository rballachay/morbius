import pvporcupine
import pyaudio
import struct
from config import PICO_ACCESS_KEY, PORCUPINE_MODEL_MAC, PORCUPINE_MODEL_PI
import platform
from controller import VoiceController, RasaManager
import subprocess

def get_porcupine_model():
    system = platform.system()
    machine = platform.machine()

    if system == "Darwin":
        return PORCUPINE_MODEL_MAC
    elif system == "Linux":
        if machine in ["armv7l", "armv6l", "aarch64"]:
            return PORCUPINE_MODEL_PI
    raise Exception("Cannot load daemon on non mac/raspberry pi")

def speak(text):
    try:
        # Call espeak with the provided text
        subprocess.run(['espeak', text], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while calling espeak: {e}")

class Daemon:
    """
    Manager for the 'daemon', aka the process that can run headless, and 
    listens for the key phrase 'Hey robot'. Contains two parts:

    1. Keyword model Porcupine which listens for 'Hey robot' 
    2. Voice controller with rasa manager which is launched when hey robot is said
    """
    def __init__(self):
        self.rasa = RasaManager()

        _keyword_path = get_porcupine_model()
        self.porcupine = pvporcupine.create(
            access_key=PICO_ACCESS_KEY, keyword_paths=[_keyword_path]
        )

        self.pa = None
        self.audio_stream = None

    # Function to run the daemon process
    def listen(self):
        print("Listening for 'Hey Robot'...")
        speak("Daemon launched")

        try:
            while True:
                self.start_audio()
                pcm = self.audio_stream.read(self.porcupine.frame_length)
                pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

                keyword_index = self.porcupine.process(pcm)
                if keyword_index >= 0:
                    pa, audio_stream = self.close_audio(pa, audio_stream)
                    self.run()
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.close_audio()
            self.porcupine.delete()
            self.rasa.rasa_controller.terminate()
            self.rasa.action_controller.terminate()

    def run(self):
        print("Hey Robot detected! Triggering full process...")
        vc = VoiceController(daemon=True, rasa=self.rasa)
        vc.run()

    def start_audio(self):
        self.pa = pyaudio.PyAudio()

        self.audio_stream = self.pa.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length,
        )

    def close_audio(self):
        if not self.audio_stream is None:
            self.audio_stream.close()
        if not self.pa is None:
            self.pa.terminate()
        return None, None


if __name__ == "__main__":
    daemon = Daemon()
    daemon.listen()
