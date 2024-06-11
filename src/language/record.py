import pathlib
import ctypes
from config import RECORD_LENGTH, SILENCE_LENGTH, VAD_MODE, SAMPLE_RATE
import os
import numpy as np

def record_until_thresh():
    """Create the folders leading to filename, then record audio to file. Will start
    audio recording, then record until silence threshold is met, then stop and return.
    """

    # Load the shared library
    libfile = pathlib.Path(__file__).parent / "record_audio.so"
    lib = ctypes.CDLL(str(libfile))

    # Define the argument and return types of the function
    lib.record.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.record.restype = ctypes.c_int

    array = np.zeros(RECORD_LENGTH*SAMPLE_RATE, dtype=np.float32)
    array_ctypes = array.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

    # Call the main function
    result = lib.record(
        SAMPLE_RATE, array_ctypes, RECORD_LENGTH, SILENCE_LENGTH, VAD_MODE
    )
    
    if not result==0:
        raise Exception("Failed to record audio using record_audio.so")

    # will return array of non-zero elements
    return array[:np.argmax(array == 0)]