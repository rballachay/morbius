import sys
sys.path.append('submodules/StyleTTS2')
from models import load_ASR_models, load_F0_models, build_model
from munch_utils import recursive_munch
from Utils.PLBERT.util import load_plbert
from Modules.diffusion.sampler import DiffusionSampler, ADPM2Sampler, KarrasSchedule
from text_utils import TextCleaner

# load packages
import time
import yaml
import phonemizer
import torch
from nltk.tokenize import word_tokenize
import torch
import numpy as np

from config import DEVICE, LJ_MODEL, LJ_CONFIG

class StyleTTS2:
    def __init__(self):
        self.sampling_rate = 24000
        
        config = yaml.safe_load(open(LJ_CONFIG))

        for key,value in config.items():
            if isinstance(value,str):
                if value.startswith('Utils/'):
                    config[key] =  f"submodules/StyleTTS2/{value}"

        # load pretrained ASR model
        ASR_config = config.get('ASR_config', False)
        ASR_path = config.get('ASR_path', False)
        text_aligner = load_ASR_models(ASR_path, ASR_config)

        # load pretrained F0 model
        F0_path = config.get('F0_path', False)
        pitch_extractor = load_F0_models(F0_path)

        # load BERT model
        BERT_path = config.get('PLBERT_dir', False)
        plbert = load_plbert(BERT_path)

        self.phonemizer = phonemizer.backend.EspeakBackend(language='en-us', preserve_punctuation=True,  with_stress=True)

        self.model = build_model(recursive_munch(config['model_params']), text_aligner, pitch_extractor, plbert)
        _ = [self.model[key].eval() for key in self.model]
        _ = [self.model[key].to(DEVICE) for key in self.model]

        params_whole = torch.load(LJ_MODEL, map_location=DEVICE)
        params = params_whole['net']

        for key in self.model:
            if key in params:
                print('%s loaded' % key)
                try:
                    self.model[key].load_state_dict(params[key])
                except:
                    from collections import OrderedDict
                    state_dict = params[key]
                    new_state_dict = OrderedDict()
                    for k, v in state_dict.items():
                        name = k[7:] # remove `module.`
                        new_state_dict[name] = v
                    # load params
                    self.model[key].load_state_dict(new_state_dict, strict=False)
                    
        _ = [self.model[key].eval() for key in self.model]

        self.sampler = DiffusionSampler(
            self.model.diffusion.diffusion,
            sampler=ADPM2Sampler(),
            sigma_schedule=KarrasSchedule(sigma_min=0.0001, sigma_max=3.0, rho=9.0), # empirical parameters
            clamp=False
        )

        self.textcleaner = TextCleaner()

    def __call__(self, text, diffusion_steps=2, embedding_scale=1):
        noise = torch.randn(1,1,256).to(DEVICE)
        text = text.strip()
        text = text.replace('"', '')
        ps = self.phonemizer.phonemize([text])

        if not ps:
            return np.zeros(0)
        
        ps = word_tokenize(ps[0])
        ps = ' '.join(ps)

        tokens = self.textcleaner(ps)
        tokens.insert(0, 0)
        tokens = torch.LongTensor(tokens).to(DEVICE).unsqueeze(0)
        
        with torch.no_grad():
            input_lengths = torch.LongTensor([tokens.shape[-1]]).to(tokens.device)
            text_mask = length_to_mask(input_lengths).to(tokens.device)

            t_en = self.model.text_encoder(tokens, input_lengths, text_mask)
            bert_dur = self.model.bert(tokens, attention_mask=(~text_mask).int())
            d_en = self.model.bert_encoder(bert_dur).transpose(-1, -2) 

            s_pred = self.sampler(noise, 
                embedding=bert_dur[0].unsqueeze(0), num_steps=diffusion_steps,
                embedding_scale=embedding_scale).squeeze(0)

            s = s_pred[:, 128:]
            ref = s_pred[:, :128]

            d = self.model.predictor.text_encoder(d_en, s, input_lengths, text_mask)

            x, _ = self.model.predictor.lstm(d)
            duration = self.model.predictor.duration_proj(x)
            duration = torch.sigmoid(duration).sum(axis=-1)
            pred_dur = torch.round(duration.squeeze()).clamp(min=1)

            pred_dur[-1] += 5

            pred_aln_trg = torch.zeros(input_lengths, int(pred_dur.sum().data))
            c_frame = 0
            for i in range(pred_aln_trg.size(0)):
                pred_aln_trg[i, c_frame:c_frame + int(pred_dur[i].data)] = 1
                c_frame += int(pred_dur[i].data)

            # encode prosody
            en = (d.transpose(-1, -2) @ pred_aln_trg.unsqueeze(0).to(DEVICE))
            F0_pred, N_pred = self.model.predictor.F0Ntrain(en, s)
            out = self.model.decoder((t_en @ pred_aln_trg.unsqueeze(0).to(DEVICE)), 
                                    F0_pred, N_pred, ref.squeeze().unsqueeze(0))

        return out.squeeze().cpu().numpy()

def length_to_mask(lengths):
    mask = torch.arange(lengths.max()).unsqueeze(0).expand(lengths.shape[0], -1).type_as(lengths)
    mask = torch.gt(mask+1, lengths.unsqueeze(1))
    return mask

if __name__=="__main__":
    text = ''' StyleTTS 2 is a text-to-speech model that leverages style diffusion and adversarial training with large speech language models to achieve human-level text-to-speech synthesis. '''
    style = StyleTTS2()
    start = time.time()
    wav = style(text, diffusion_steps=5, embedding_scale=1)
    rtf = (time.time() - start) / (len(wav) / 24000)
    print(f"RTF = {rtf:5f}")