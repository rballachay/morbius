from src.language.record import record_until_thresh
import numpy as np
import subprocess

data=record_until_thresh()

print(data.size)

# Convert float32 array to int16 array
float32_array = np.array(data, dtype=np.float32)
int16_array = (float32_array * 32767).astype(np.int16)

audio_data_bytes = int16_array.tobytes()
command = ['aplay', '-f', 'S16_LE', '-r', f'8000', '-c', '2']

with subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE) as proc:
    # Write the converted audio data to stdin
    proc.stdin.write(audio_data_bytes)
    proc.stdin.close()
    proc.wait()
