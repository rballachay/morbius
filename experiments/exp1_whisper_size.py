from src.language.transcribe import Whisper, FasterWhisper
from src.language.tts.text_to_speech import TextToSpeech
import yaml
import re
from config import WHISPER_SIZES
from scipy.signal import resample
import numpy as np
import pyaudio
import pickle
from tqdm import tqdm
import os
from .utils import calculate_wer
import time
import pandas as pd

AUDIO_CACHE = 'experiments/exp1-audio-cached.pkl'
RESULTS_FILE = 'experiments/results/exp1_results.csv'

def speak(data_16k):
    pyaudio_stream = pyaudio.PyAudio()
    stream = pyaudio_stream.open(format=pyaudio.paFloat32,
                        channels=1,
                        rate=16000,
                        output=True,
                        output_device_index=1
                        )   
    stream.write(data_16k.astype(np.float32).tostring())

# Function to load YAML file
def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def parse_data(file='data/rasa/v1-full/data/nlu.yml'):
    data = load_yaml(file)['nlu']
    
    total_quotes = []

    for value in data:
        intent = value['intent'] # not really necessary
        examples = value['examples']
        for example in examples.split('\n'):
            example=example.replace('[',' ')
            example=example.replace(']',' ')
            example=example.replace('- ','')
            pattern = re.compile(r'\{[^}]*\}')
            # Replace the matched text with a space
            example = pattern.sub('', example)

            #if example=='':
            #    continue

            total_quotes.append(example)

    return total_quotes

def gen_audio(total_quotes):
    audio_list= []

    tts = TextToSpeech(active_tts='styleTTS2')

    for quote in tqdm(total_quotes):
        # whisper expects the audio sample rate to be 16khz
        try:
            audio = tts.model(quote)
        except:
            print(f"Failed vocalizing \'{quote}\', continuing...")

        # Calculate the number of samples in the resampled data
        num_samples = int(len(audio) * 16000 / tts.model.sampling_rate)

        # Resample the data
        try:
            data_16k = resample(audio, num_samples)
        except:
            print(f"Failed resampling \'{quote}\', continuing...")

        audio_list.append(data_16k)
        
        "uncomment to test that audio sounds ok"
        #speak(data_16k)
        
    return audio_list


def run():
    quotes = parse_data()

    # save the audio to a pkl file so it can be used later 
    if os.path.exists(AUDIO_CACHE):
        with open(AUDIO_CACHE,'rb') as f:
            generated_audio = pickle.load(f)
    else:
        generated_audio = gen_audio(quotes)

        with open(AUDIO_CACHE,'wb') as f:
            pickle.dump(generated_audio, f)
    
    results={'size':[],'wer':[],'speed':[],'quotes':[],'nwords':[],'class':[]}

    for whisper_size in WHISPER_SIZES:

        for whisper_class in (FasterWhisper, Whisper):
            whisper = whisper_class(whisper_size)

            for quote, audio in zip(quotes,generated_audio):
                start = time.time()
                whisper_quote = whisper.transcribe(audio)
                end=time.time()

                wer, nwords = calculate_wer(quote, whisper_quote)

                time_elapsed=end-start
                print(f"Time: {time_elapsed}")
                print(f"{quote} | {whisper_quote} | {wer}")

                results['quotes'].append(quote)
                results['wer'].append(wer)
                results['nwords'].append(nwords)
                results['speed'].append(time_elapsed)
                results['size'].append(whisper_size)
                results['class'].append(type(whisper).__name__)

    pd.DataFrame(results).to_csv(RESULTS_FILE,index=False)

if __name__ == "__main__":
    run()
