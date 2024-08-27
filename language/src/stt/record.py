import pathlib
import ctypes
from config import RECORD_LENGTH, SILENCE_LENGTH, VAD_MODE, SAMPLE_RATE
import numpy as np
import os
import platform
from src.file_utils import install_on_linux, install_on_mac, is_tool_installed, update_submodule
import subprocess

def record_until_thresh():
    """Create the folders leading to filename, then record audio to file. Will start
    audio recording, then record until silence threshold is met, then stop and return.

    Note that this uses ctypes, which is a library which we are using to call
    a c script from python, as c contains the voice activity detection library
    that is best for detecting silence.
    """

    # Load the shared library
    libfile = pathlib.Path(__file__).parent / "record_audio.so"

    # attempt to compile shared library if it doesnt exist
    if not os.path.exists(libfile):
        print("record_audio.so doesn't exist. Will attempt to compile")
        __install_deps()
        # compile the actual file
        result = subprocess.run(['gcc', '-shared', '-o', 'record_audio.so', '-fPIC', 'record_audio.c', '-lportaudio', '-lfvad'],
                        check=True,cwd='src/language', capture_output=True, text=True)
        print(result.stdout)  # Print any output from the compilation process
        print(result.stderr)

    # load c function into python
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
    print(array)
    return np.trim_zeros(array)

def __install_deps():
    os_name = platform.system()

    installer = {
        'Linux':install_on_linux,
        'Darwin':install_on_mac,
    }.get(os_name, None)

    shared_lib = {
        'Linux':'so',
        'Darwin':'dylib'
    }.get(os_name,None)

    if installer is None:
        raise Exception(f"{os_name} not supported")

    # these are simple as they're available from brew/apt
    for tool in ['espeak','portaudio']:
        if not is_tool_installed(tool):
            installer(tool)

    # this package needs to be installed from github
    if not os.path.exists(f'/usr/local/lib/libfvad.{shared_lib}'):
        if not is_tool_installed('autoconf'):
            installer('autoconf')

        if not is_tool_installed('libtool'):
            installer('libtool')

        if not is_tool_installed('pkg-config'):
            installer('pkg-config')

        if os_name=='Darwin':
            if not is_tool_installed('automake'):
                installer('automake')

        submodule_dir='submodules/libfvad'
        update_submodule(submodule_dir)
        subprocess.run(['autoreconf', '-i'], check=True, cwd=submodule_dir)
        subprocess.run(['./configure'], check=True, cwd=submodule_dir)
        subprocess.run(['make'], check=True, cwd=submodule_dir)
        subprocess.run(['make', 'install'], check=True, cwd=submodule_dir)