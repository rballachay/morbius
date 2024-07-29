import ctypes
import pathlib
import numpy as np


class ESpeak:
    def __init__(self):
        self.sampling_rate = 22050
        self.max_intensity = 32768

        # Load the shared library
        libfile = pathlib.Path(__file__).parent / "speak_audio.so"
        self.lib = ctypes.CDLL(str(libfile))

        # Define the argument and return types for say_text function
        self.lib.say_text.restype = ctypes.POINTER(ctypes.c_short)
        self.lib.say_text.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_size_t)]

    def __call__(self,text):
        output_size = ctypes.c_size_t()
        text_bytes = text.encode('utf-8')
        
        # Call the say_text function
        audio_data_ptr = self.lib.say_text(text_bytes, ctypes.byref(output_size))
        
        # Convert the raw audio data to a NumPy array
        if audio_data_ptr:
            audio_data = np.ctypeslib.as_array(audio_data_ptr, shape=(output_size.value,))
            return audio_data/self.max_intensity
        else:
            return None