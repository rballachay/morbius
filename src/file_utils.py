import os
import requests


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
