from src.language.transcribe import listen_transcribe
from src.language.whisper import Whisper
from config import RASA_MODEL_PATH, RASA_ACTIONS_PATH, LOG_PATH, RASA_PORT, ACTIONS_PORT
import subprocess
import time
import os
import requests
import uuid

# silent, for development, allows us to write in verbal commands via
# the shell instead of saying them, to make it easier
SILENT=True

class VoiceController:
    """Object for orchestrating voice dialogue system. 
    
    """
    def __init__(self, rasa_model_path=RASA_MODEL_PATH, rasa_action_path=RASA_ACTIONS_PATH, 
                 whisper_size='tiny', log_path=LOG_PATH, rasa_port=RASA_PORT, actions_port=ACTIONS_PORT):
        
        # create logging
        rasa_logs = f"{log_path}/rasa-logs-{int(time.time())}.log"
        dialogue_transcript = f"{log_path}/dialogue-transcript-{int(time.time())}.log"

        os.makedirs(os.path.dirname(rasa_logs), exist_ok=True)
        
        # init everything related to rasa + then wait to launch
        self.rasa_url = f"http://0.0.0.0:{rasa_port}"
        self.actions_url = f"http://0.0.0.0:{actions_port}"
        self.rasa_port = rasa_port
        self.actions_port=actions_port
        self.rasa_controller, self.action_controller = self.__run_rasa(rasa_model_path, 
                                                                     rasa_action_path, rasa_logs,
                                                                     rasa_port, actions_port)


        self.whisper_agent = Whisper(whisper_size)

    
    def run(self):
        while True:
            input("Press Enter to start a conversation...")
            conversation = Conversation(self.rasa_url)

            # w
            while conversation.open():
                if SILENT:
                    results = input("(dev mode) Type next command: ")
                else:
                    results = listen_transcribe(self.whisper_agent)

                

    def __run_rasa(self, model_path, action_path, rasa_logs, rasa_port, actions_port):
        """Launches the two rasa processes: action server and dialogue server.
        """
        rasa_cmd = f"rasa run --enable-api -p {rasa_port} --model {model_path}"
        action_cmd = f"rasa run actions --actions {action_path} -p {actions_port}"

        with open(rasa_logs, 'a') as log_file:
            rasa_process = subprocess.Popen(rasa_cmd, shell=True, stdout=subprocess.PIPE, stderr=log_file)
            print(f"Rasa server launched with PID: {rasa_process.pid}")

            action_process = subprocess.Popen(action_cmd, shell=True, stdout=subprocess.PIPE, stderr=log_file)
            print(f"Rasa action server launched with PID: {action_process.pid}")

            self.__await_rasa_launch()

            return rasa_process, action_process
        
    def __await_rasa_launch(self):
        """subprocess.Popen will launch the subprocess. 
        Need to wait until the endpoints are active before starting 
        the rest of the controller.
        """
        rasa_status=False
        actions_status=False
        while True:
            try:
                if not rasa_status:
                    response = requests.get(self.rasa_url+'/status')
                    if response.status_code==200:
                        rasa_status=True

                if not actions_status:
                    response = requests.get(self.actions_url+'/health')
                    if response.status_code==200:
                        actions_status=True
            except:
                print(f"Finishing launching rasa servers...")

            if rasa_status and actions_status:
                break
            
            time.sleep(1)

class Conversation:
    """Manages a single conversation + the state, without 
    needing to reference the 
    """
    def __init__(self, rasa_url):
        # uuid is used as the 
        self.uuid=uuid.uuid4().hex
        self.rasa_url = rasa_url

        self.message_history=[]
        self.resposne_history=[]

    def __call__(self, message=""):

        url = self.rasa_url+'/webhooks/rest/webhook'

        response = requests.post(url,json={"message":message,"sender":self.uuid})

        if not response.status_code==200:
            raise Exception("Failed calling rasa endpoint")
    
        results = response.json()["text"]

        return results





    

if __name__ == "__main__":
    vc=None
    try:
        vc = VoiceController()

        vc.run()
    finally:
        if vc is not None:
            vc.rasa_controller.terminate()
            vc.action_controller.terminate()
    