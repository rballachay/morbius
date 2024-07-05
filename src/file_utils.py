import os
import requests
import time
from functools import wraps
import gdown
import zipfile
import re
import subprocess

def download_file(url, filename):
    """Download file (usually model) from url to local filename with
    path. Make nested directory of destination if it doesn't exist.
    """

    if os.path.exists(filename):
        print(f"{filename} already exists, skipping download")
        return False

    # make directory
    os.makedirs(filename.split("/")[0], exist_ok=True)

    response = requests.get(url, stream=True)

    try:
        if response.status_code == 200:
            with open(filename, "wb") as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)
            print(f"{filename} downloaded successfully.")
            return True
        else:
            print(f"Failed to download {filename}. Status code: {response.status_code}")
            return False
    except:
        # if it fails for some reason during the download, remove partial file
        try:
            os.remove(filename)
        except OSError:
            pass

        return False

def timing_decorator(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"Function '{func.__name__}' executed in {elapsed_time:.4f} seconds")
        return result
    return wrapper


def download_model_gdrive(link:str, filename:str, is_zip:bool=False):
    """Download a model from google drive. Not enough space on git lfs.
    """
    print(f"Couldn't find {filename} locally. Downloading from google drive")
    
    # extract the id from the url
    pattern = r"/d/([^/]+)/view"
    id = re.search(pattern, link).group(1)

    directory=filename.split("/")[0]
    os.makedirs(directory, exist_ok=True)

    url = f'https://drive.google.com/uc?id={id}'

    # some of the filenames will be a destination folder, so we 
    # want to extract everything to that path
    if is_zip:
        gdown.download(url, '.temp.zip', quiet=False)
        with zipfile.ZipFile('.temp.zip', 'r') as zip_ref:
            zip_ref.extractall(directory)
        os.remove('.temp.zip')
    else:
        gdown.download(url, filename, quiet=False)

def is_tool_installed(tool):
    """Check if a tool is installed."""
    try:
        result = subprocess.run(['which', tool], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return result.returncode == 0
    except Exception as e:
        print(f"An error occurred while checking for {tool}: {e}")
        return False

def install_on_linux(package):
    """Install portaudio and espeak on Linux using apt-get."""
    try:
        subprocess.run(['sudo', 'apt-get', 'update'], check=True)
        subprocess.run(['sudo', 'apt-get', 'install', '-y', package], check=True)
        print(f"Successfully installed {package} on Linux.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred during installation on Linux: {e}")

def install_on_mac(package):
    """Install portaudio and espeak on macOS using brew."""
    try:
        subprocess.run(['brew', 'update'], check=True)
        subprocess.run(['brew', 'install', package], check=True)
        print(f"Successfully installed {package} on macOS.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred during installation on macOS: {e}")

def update_submodule(submodule_path):
    try:
        # Navigate to the submodule directory
        subprocess.run(['git', 'submodule', 'update', '--init', submodule_path], check=True)
        print(f"Successfully updated submodule at {submodule_path}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while updating the submodule: {e}")