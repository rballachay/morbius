import pathlib
import ctypes
from config import RECORD_LENGTH, SILENCE_LENGTH, SILENCE_THRESH
import os


async def record_until_thresh(filename):
    """Create the folders leading to filename, then record audio to file. Will start
    audio recording, then record until silence threshold is met, then stop and save
    to file.
    """
    await _create_subfolders_for_file(filename)

    # Load the shared library
    libfile = pathlib.Path(__file__).parent / "record_audio.so"
    lib = ctypes.CDLL(str(libfile))

    # Define the argument and return types of the function
    lib.record.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_float, ctypes.c_int]
    lib.record.restype = ctypes.c_int

    # Call the main function
    result = lib.record(
        filename.encode("utf-8"), RECORD_LENGTH, SILENCE_THRESH, SILENCE_LENGTH
    )

    # will return zero if success
    return result == 0


async def _create_subfolders_for_file(file_path):
    directory = os.path.dirname(file_path)

    if directory and not os.path.exists(directory):
        os.makedirs(directory)
        print(f"Directories created for {directory}")
    else:
        print(f"Directory {directory} already exists")
