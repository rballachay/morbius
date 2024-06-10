# add the submodules/FastSpeech2 path so that we 
# can import model and hifigan normally from here
import sys
sys.path.append('submodules/FastSpeech2')
from model import FastSpeech2
import hifigan as hifigan
from text import text_to_sequence
from utils.tools import to_device
from utils.model import vocoder_infer

# other packages
import time
from g2p_en import G2p
import re
from string import punctuation
import yaml
import json
import torch
import zipfile
import os
import numpy as np

# this config is imported from the base path
from config import PREPROCESS_CONFIG, MODEL_CONFIG, \
    DEVICE, HIFIGAN_CONFIG, HIFIGAN_MODEL, FAST_MODEL_PATH

class FastSpeech:
    def __init__(self, model_path=FAST_MODEL_PATH):
        self.model, self.model_config,  \
            self.preprocess_config = self.__get_model(model_path)
        
        self.vocoder = self.__get_vocoder()

        # this should be the case unless it is changed
        self.sampling_rate = 22050
    
    def __call__(self, message):
        pitch_control, energy_control, duration_control = (1.0, 1.0, 1.0) 
        speakers = np.array([0])

        ids = raw_texts = [message[:100]]
        texts = np.array([preprocess_english(message, self.preprocess_config)])
        text_lens = np.array([len(texts[0])])
        batchs = [(ids, raw_texts, speakers, texts, text_lens, max(text_lens))]

        for batch in batchs:
            batch = to_device(batch, DEVICE)
            with torch.no_grad():
                # Forward
                predictions = self.model(
                    *(batch[2:]),
                    p_control=pitch_control,
                    e_control=energy_control,
                    d_control=duration_control
                )

                mel_predictions = predictions[1].transpose(1, 2)
                lengths = predictions[9] * self.preprocess_config["preprocessing"]["stft"]["hop_length"]
                
                wav_predictions = vocoder_infer(
                    mel_predictions, self.vocoder, self.model_config, 
                    self.preprocess_config, lengths=lengths
                )

                self.sampling_rate = self.preprocess_config["preprocessing"]["audio"]["sampling_rate"]
                max_value =  self.preprocess_config["preprocessing"]["audio"]["max_wav_value"]

                wav_predictions=np.array(wav_predictions).squeeze()/max_value
        
        return wav_predictions

    def __get_model(self, model_path):
        preprocess_config = yaml.load(
            open(PREPROCESS_CONFIG, "r"), Loader=yaml.FullLoader
        )
        self.preprocess_config = self.__change_cfg(preprocess_config)

        model_config = yaml.load(open(MODEL_CONFIG, "r"), Loader=yaml.FullLoader)

        model = FastSpeech2(preprocess_config, model_config).to(DEVICE)
        model.load_state_dict(torch.load(model_path, map_location=DEVICE)['model'])
        model.eval()
        model.requires_grad_ = False
        return model, model_config, preprocess_config
    
    def __get_vocoder(self):
        with open(HIFIGAN_CONFIG, "r") as f:
            config = json.load(f)
        config = hifigan.AttrDict(config)
        vocoder = hifigan.Generator(config)

        # by default, this is downloaded as a zip file, so it 
        # needs to be extracted in order to work properly
        if not os.path.exists(HIFIGAN_MODEL):
            with zipfile.ZipFile(HIFIGAN_MODEL+'.zip', 'r') as zip_ref:
                zip_ref.extractall('submodules/FastSpeech2/hifigan')

        ckpt = torch.load(HIFIGAN_MODEL,map_location=DEVICE)
        vocoder.load_state_dict(ckpt["generator"])
        vocoder.eval()
        vocoder.remove_weight_norm()
        vocoder.to(DEVICE)
        return vocoder
    
    def __change_cfg(self, config):
        config['path']['preprocessed_path']='submodules/FastSpeech2/preprocessed_data/LJSpeech'
        config['path']['lexicon_path']='submodules/FastSpeech2/lexicon/librispeech-lexicon.txt'
        return config

#### Additional funtions that are from :
# https://github.com/ming024/FastSpeech2/blob/master/synthesize.py

def preprocess_english(text, preprocess_config):
    text = text.rstrip(punctuation)
    lexicon = read_lexicon(preprocess_config["path"]["lexicon_path"])

    g2p = G2p()
    phones = []
    words = re.split(r"([,;.\-\?\!\s+])", text)
    for w in words:
        if w.lower() in lexicon:
            phones += lexicon[w.lower()]
        else:
            phones += list(filter(lambda p: p != " ", g2p(w)))
    phones = "{" + "}{".join(phones) + "}"
    phones = re.sub(r"\{[^\w\s]?\}", "{sp}", phones)
    phones = phones.replace("}{", " ")

    sequence = np.array(
        text_to_sequence(
            phones, preprocess_config["preprocessing"]["text"]["text_cleaners"]
        )
    )

    return np.array(sequence)

def read_lexicon(lex_path):
    lexicon = {}
    with open(lex_path) as f:
        for line in f:
            temp = re.split(r"\s+", line.strip("\n"))
            word = temp[0]
            phones = temp[1:]
            if word.lower() not in lexicon:
                lexicon[word.lower()] = phones
    return lexicon


if __name__=="__main__":
    text = ''' StyleTTS 2 is a text-to-speech model that leverages style diffusion and adversarial training with large speech language models to achieve human-level text-to-speech synthesis. '''
    fastspeech = FastSpeech()
    start = time.time()
    noise = torch.randn(1,1,256).to(DEVICE)
    wav = fastspeech(text)
    rtf = (time.time() - start) / (len(wav) / fastspeech.sampling_rate)
    print(f"RTF = {rtf:5f}")