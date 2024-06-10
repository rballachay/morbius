# add the submodules/FastSpeech2 path so that we 
# can import model and hifigan normally from here
import sys
sys.path.append('submodules/FastSpeech2')
from model import FastSpeech2
import hifigan as hifigan

import yaml
import json
import torch
import zipfile
import os

# this config is imported from the base path
from config import PREPROCESS_CONFIG, MODEL_CONFIG, \
    DEVICE, HIFIGAN_CONFIG, HIFIGAN_MODEL, FAST_MODEL_PATH

class FastSpeech:
    def __init__(self, model_path=FAST_MODEL_PATH):
        self.model = self.__get_model(model_path)
        self.vocoder = self.__get_vocoder()

        control_values = (1.0, 1.0, 1.0) # pitch, energy, duration


    def __get_model(self, model_path):
        preprocess_config = yaml.load(
            open(PREPROCESS_CONFIG, "r"), Loader=yaml.FullLoader
        )
        preprocess_config = self.__change_cfg(preprocess_config)

        model_config = yaml.load(open(MODEL_CONFIG, "r"), Loader=yaml.FullLoader)
        
        model_path = model_path 

        model = FastSpeech2(preprocess_config, model_config).to(DEVICE)
        model.load_state_dict(torch.load(FAST_MODEL_PATH, map_location=DEVICE)['model'])
        model.eval()
        model.requires_grad_ = False
        return model
    
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
        return config
        

