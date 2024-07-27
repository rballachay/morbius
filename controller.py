from src.language.transcribe import FasterWhisper, listen_transcribe
from src.file_utils import timing_decorator
from src.ros_controller import RosControllerV1, RosControllerv2
from config import RASA_MODEL_PATHS, RASA_MODELS_GDRIVE, RASA_ACTIONS_PATHS, \
    LOG_PATH, RASA_PORT, ACTIONS_PORT, WHISPER_SIZE, RASA_VERSION, SILENT_IN, \
          SILENT_OUT, SILENT_ROBOT
import subprocess
import time
import os
import requests
import uuid
from src.language.tts.text_to_speech import TextToSpeech
from src.file_utils import download_model_gdrive


class RasaManager:
    """Class for mananing launching of rasa service. 
    There are two services: the action and the model. These are 
    served on the local network at port ACTIONS_PORT and RASA_PORT, 
    respectively. This is launched separately from the voice controller
    as it takes a while to launch, so it will be always be active in the 
    daemon, and the text-to-voice and voice-to-text will be launched 
    in the VoiceController object.

    rasa_version: str [v1, v2] -> See data/rasa/README.md
    rasa_model_paths: {v1:str, v2:str} 
    rasa_models_grdive: {v1:str, v2:str}
    rasa_action_paths: {v1:str, v2:str}
    log_path: str 
    rasa_port: int
    actions_port: int
    """
    def __init__(
        self,
        rasa_version=RASA_VERSION, # v1 or v2
        rasa_model_paths=RASA_MODEL_PATHS, # dictionary
        rasa_models_gdrive=RASA_MODELS_GDRIVE, # dictionary
        rasa_action_paths=RASA_ACTIONS_PATHS, # dictionary
        log_path=LOG_PATH,
        rasa_port=RASA_PORT,
        actions_port=ACTIONS_PORT,
    ):  
        
        self.rasa_version = rasa_version

        if not self.rasa_version in ['v1','v2']:
            raise Exception("Rasa version must be one of: {v1, v2}")

        # select rasa paths based on version
        rasa_model_path = rasa_model_paths[rasa_version]
        rasa_model_gdrive = rasa_models_gdrive[rasa_version]
        rasa_action_path = rasa_action_paths[rasa_version]

        # rasa models are stored in google drive. this checks if
        # they exist prior to trying to launch the agent
        if not os.path.exists(rasa_model_path):
            download_model_gdrive(rasa_model_gdrive,rasa_model_path)

        # data.rasa.actions -> data/rasa/enpdpoints.yml
        rasa_endpoint_path = rasa_action_path.replace('.','/').replace('actions','endpoints.yml')

        # create logging
        rasa_logs = f"{log_path}/rasa-logs-{int(time.time())}.log"
        #dialogue_transcript = f"{log_path}/dialogue-transcript-{int(time.time())}.log"

        os.makedirs(os.path.dirname(rasa_logs), exist_ok=True)

        # init everything related to rasa + then wait to launch
        self.rasa_url = f"http://0.0.0.0:{rasa_port}"
        self.actions_url = f"http://0.0.0.0:{actions_port}"
        self.rasa_port = rasa_port
        self.actions_port = actions_port
        self.rasa_controller, self.action_controller = self.__run_rasa(
            rasa_model_path, rasa_action_path, rasa_endpoint_path, rasa_logs, rasa_port, actions_port
        )

    def __run_rasa(self, model_path, action_path, endpoint_path, rasa_logs, rasa_port, actions_port):
        """
        Launches the two rasa processes: action server and dialogue server.
        Subprocess is essentially running a shell command from inside python. 
        This launches a separate process with a separate PID. By keeping the pipe,
        we can close the process when ending rasa.
        """
        rasa_cmd = f"rasa run --enable-api -p {rasa_port} --model {model_path} --endpoints {endpoint_path}"
        action_cmd = f"rasa run actions --actions {action_path} -p {actions_port}"

        with open(rasa_logs, "a") as log_file:
            rasa_process = subprocess.Popen(
                rasa_cmd, shell=True, stdout=subprocess.PIPE, stderr=log_file
            )
            self.print(f"Rasa server launched with PID: {rasa_process.pid}")

            action_process = subprocess.Popen(
                action_cmd, shell=True, stdout=subprocess.PIPE, stderr=log_file
            )
            self.print(f"Rasa action server launched with PID: {action_process.pid}")

            self.__await_rasa_launch()

            return rasa_process, action_process

    def __await_rasa_launch(self):
        """subprocess.Popen will launch the subprocess.
        Need to wait until the endpoints are active before starting
        the rest of the controller.
        """
        rasa_status = False
        actions_status = False
        while True:
            try:
                if not rasa_status:
                    response = requests.get(self.rasa_url + "/status")
                    if response.status_code == 200:
                        rasa_status = True

                if not actions_status:
                    response = requests.get(self.actions_url + "/health")
                    if response.status_code == 200:
                        actions_status = True
            except:
                self.print(f"Finishing launching rasa servers...")

            if rasa_status and actions_status:
                break

            time.sleep(2)

    def print(self, message):
        # this could change in the future if we want to
        print(message)


class Conversation:
    """Manages a single conversation + the state, without
    needing to reference anything from the api endpoint.
    """

    def __init__(self, rasa_url):
        # uuid is used as the sender id, so
        # that each conversation is unique
        self.uuid = uuid.uuid4().hex
        self.rasa_url = rasa_url

        self.message_history = []
        self.resposne_history = []

        self.state = True

    @timing_decorator
    def __call__(self, message):

        url = self.rasa_url + "/webhooks/rest/webhook"

        response = requests.post(url, json={"message": message, "sender": self.uuid})

        if not response.status_code == 200:
            raise Exception("Failed calling rasa endpoint")

        # add to messages
        self.message_history.append(message)

        message_out, actions_out = self.__parse_response(response.json())

        # add to responses
        self.resposne_history.append(message_out)

        return message_out, actions_out

    def __parse_response(self, response):
        """the api returns a list of dictionaries like the following:
        [{'recipient_id': '7e5b5f0c98a94bee937741575092e63d', \
            'text': 'Okay! Starting task...'}, {'recipient_id': '7e5b5f0c98a94bee937741575092e63d', \
                'custom': {'robot_action': 'custom_action'}}]]
        """

        message_out = []
        actions_out = []

        for sub_obj in response:
            if "text" in sub_obj.keys():
                message_out.append(sub_obj["text"])

            if "custom" in sub_obj.keys():
                actions_out.append(sub_obj["custom"])

        return message_out, actions_out

    def open(self):
        # check if state is open
        return self.state

    def close_conversation(self):
        # set state to false so that open returns false
        self.state = False


class VoiceController:
    """Object for orchestrating voice dialogue system. 
    Contains four main components:
    
    1. Voice-to-text model (whisper/fasterwhisper)
    2. Intent and entity extraction model (Rasa endpoint)  
    3. Mapper from rasa response to ros controller
    4. Text-to-speech model (nix tts/other models in config)

    Controls the entire voice pipeline workflow. 
    """

    def __init__(
        self,
        daemon=False, # determines if the process quits after
        whisper_size=WHISPER_SIZE,
        rasa=RasaManager
    ):
        self.daemon = daemon
        self.rasa = rasa

        self.whisper_agent = FasterWhisper(whisper_size)
        self.whisper_agent.transcribe = timing_decorator(self.whisper_agent.transcribe)

        # the controller depends on the version of rasa, as the actions 
        # are mapped to the ROS controller
        self.ros_controller = {
            'v1':RosControllerV1,
            'v2':RosControllerv2
        }[self.rasa.rasa_version]()

        self.text_to_speech = TextToSpeech()
        self.text_to_speech.speak = timing_decorator(self.text_to_speech.speak)
        

    def run(self):
        """This is a loop that continues until it is shut down. When
        a conversation is finished, you can press enter to re-start.
        A conversation will prompt for input until it is asked to sleep.
        """

        while True:
            # if daemon, we are running headless
            if self.daemon:
                self.print("Hello there, how can I help you?")
                if not SILENT_OUT:
                    self.text_to_speech.speak("Hello there, how can I help you?")
            else:
                input("Press Enter to start a conversation...")

            conversation = Conversation(self.rasa.rasa_url)

            # if its asleep, wake it up
            self.ros_controller.wake_up()

            while conversation.open():
                if SILENT_IN:
                    message_in = input("(dev mode) Type next command: ")
                else:
                    message_in = listen_transcribe(self.whisper_agent)

                message_out, actions_out = conversation(message_in)

                # report the messages
                self.print("\n".join(message_out))

                if not SILENT_OUT:
                    self.text_to_speech.speak('.'.join(message_out))

                action_messages = self.__run_actions(actions_out)

                # this is the message that is returned from ros controller
                for msg in action_messages:
                    if not msg is None:
                        if SILENT_ROBOT:
                            self.print(msg)
                        else:
                            self.text_to_speech.speak(msg)

                # if it is asleep, or some other terminal state,
                # we will close the conversation. will need to awake
                if not self.ros_controller.awake():
                    conversation.close_conversation()

                    # if daemon, exit once its told to sleep
                    if self.daemon:
                        self.print("Say 'Hey robot' to wake me back up")
                        if not SILENT_OUT:
                            self.text_to_speech.speak("Say 'Hey robot' to wake me back up")
                        return

    def __run_actions(self, actions_out):
        """Parse the actions and then run ros. Actions will be a list like:
        [{'action': 'action_move_robot', 'room': 'operating room'},
          {'action': 'action_find_object', 'object': 'stethescope'}]
        """
        status_messages=[]
        for action in actions_out:
            action_name = action.pop("action", None)
            if not action_name:
                raise Exception("Missing action title in called action")

            method = getattr(self.ros_controller, action_name, None)

            if method:
                # at this point, the rest of the action dict should have
                # the kwargs that will be passed to the action
                status_message = method(**action)
                status_messages.append(status_message)
            else:
                print(f"Method {action_name} not found in ros controller")
        
        # this can be none - in that case don't say anything
        return status_messages

    def print(self, message):
        # this could change in the future if we want to
        print(message)


if __name__ == "__main__":
    vc = None
    try:
        rm = RasaManager()
        vc = VoiceController(rasa=rm)
        vc.run()
    finally:
        if rm is not None:
            rm.rasa_controller.terminate()
            rm.action_controller.terminate()
