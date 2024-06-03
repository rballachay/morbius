import ctypes

def main(filename):
    # Load the shared library
    lib = ctypes.CDLL('./record_audio.so')

    # Define the argument and return types of the function
    lib.record.argtypes = [ctypes.c_char_p]
    lib.record.restype = ctypes.c_int

    # Call the main function
    result = lib.record(filename.encode('utf-8'))

    return result

if __name__ == '__main__':
    # Pass FILENAME as a parameter
    FILENAME = "recorded_audio_NEW.wav"
    result = main(FILENAME)
    print("Result:", result)
