import requests
from config import WHISPER_DIR, WHISPER_PATH
import os

# full range of models - we want the english-only models
MODELS = ["tiny","base","small","medium"]
MODEL_NAME_LOCAL="whisper_torch_{model_size}.bin"

def run():
    __download_models()

def __download_models():
    for model_size in MODELS:
        url = WHISPER_PATH.format(model_size=model_size)
        filename=f"{WHISPER_DIR}/{MODEL_NAME_LOCAL.format(model_size=model_size)}"
        _download_file(url,filename)
    
def _download_file(url, filename):

    if os.path.exists(filename):
        print(f"{filename} already exists, skipping download")
        return

    # make directory 
    os.makedirs(filename.split('/')[0], exist_ok=True)

    response = requests.get(url, stream=True)

    try:
        if response.status_code == 200:
            with open(filename, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)
            print(f"{filename} downloaded successfully.")
        else:
            print(f"Failed to download {filename}. Status code: {response.status_code}")
    except:
        #if it fails for some reason during the download, remove partial file
        try:
            os.remove(filename)
        except OSError:
            pass


if __name__=="__main__":
    run()