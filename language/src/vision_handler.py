from config import VISION_MODEL_PORT
import requests

class VisionHandler:
    def __init__(self, port=VISION_MODEL_PORT):
        self.url = f"http://localhost:{port}"

    def distance(self):
        return self.get().get('max_distance', None)

    def get(self)->dict:
        try:
            response = requests.get(self.url)
        except:
            print("Failed to send request")
            return {}
        
        if response.status_code!=200:
            print("Failed get request")
            return {}
        json = response.json()
        return json

