from src.language.transcribe import Whisper, listen_transcribe
from src.file_utils import timing_decorator
from config import RASA_MODEL_PATH, RASA_ACTIONS_PATH, LOG_PATH, RASA_PORT, ACTIONS_PORT
import subprocess
import time
import os
import requests
import uuid
from src.language.tts.text_to_speech import TextToSpeech

# silent, for development, allows us to write in verbal commands via
# the shell instead of saying them, to make it easier
SILENT = False

class VoiceController:
    """Object for orchestrating voice dialogue system."""

    def __init__(
        self,
        rasa_model_path=RASA_MODEL_PATH,
        rasa_action_path=RASA_ACTIONS_PATH,
        whisper_size="tiny",
        log_path=LOG_PATH,
        rasa_port=RASA_PORT,
        actions_port=ACTIONS_PORT,
    ):

        # create logging
        rasa_logs = f"{log_path}/rasa-logs-{int(time.time())}.log"
        dialogue_transcript = f"{log_path}/dialogue-transcript-{int(time.time())}.log"

        os.makedirs(os.path.dirname(rasa_logs), exist_ok=True)

        # init everything related to rasa + then wait to launch
        self.rasa_url = f"http://0.0.0.0:{rasa_port}"
        self.actions_url = f"http://0.0.0.0:{actions_port}"
        self.rasa_port = rasa_port
        self.actions_port = actions_port
        self.rasa_controller, self.action_controller = self.__run_rasa(
            rasa_model_path, rasa_action_path, rasa_logs, rasa_port, actions_port
        )

        self.whisper_agent = Whisper(whisper_size)

        self.ros_controller = RosController()

        self.text_to_speech = TextToSpeech()

    def run(self):
        """This is a loop that continues until it is shut down. When
        a conversation is finished, you can press enter to re-start.
        A conversation will prompt for input until it is asked to sleep.
        """
        while True:
            input("Press Enter to start a conversation...")
            conversation = Conversation(self.rasa_url)

            # if its asleep, wake it up
            self.ros_controller.wake_up()

            while conversation.open():
                if SILENT:
                    message_in = input("(dev mode) Type next command: ")
                else:
                    message_in = listen_transcribe(self.whisper_agent)

                message_out, actions_out = conversation(message_in)

                # report the messages
                self.print("\n".join(message_out))
                self.text_to_speech.speak('.'.join(message_out))

                self.__run_actions(actions_out)

                # if it is asleep, or some other terminal state,
                # we will close the conversation. will need to awake
                if not self.ros_controller.awake():
                    conversation.close_conversation()

    def __run_actions(self, actions_out):
        """Parse the actions and then run ros. Actions will be a list like:
        [{'action': 'action_move_robot', 'room': 'operating room'},
          {'action': 'action_find_object', 'object': 'stethescope'}]
        """

        for action in actions_out:
            action_name = action.pop("action", None)
            if not action_name:
                raise Exception("Missing action title in called action")

            method = getattr(self.ros_controller, action_name, None)

            if method:
                # at this point, the rest of the action dict should have
                # the kwargs that will be passed to the action
                method(**action)
            else:
                print(f"Method {action_name} not found in ros controller")

    def __run_rasa(self, model_path, action_path, rasa_logs, rasa_port, actions_port):
        """Launches the two rasa processes: action server and dialogue server."""
        rasa_cmd = f"rasa run --enable-api -p {rasa_port} --model {model_path} --endpoints data/rasa/endpoints.yml"
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


class RosController:
    """For the first iteration of this, it's not clear
    how the roscontroller is going to work, so i will
    make a bare-bones version of this
    """

    def __init__(self):
        self.state = True

    def action_move_robot(self, room):
        self.print(f"Moving robot to room {room}")

    def action_find_object(self, object):
        self.print(f"Searching for object {object}")

    def action_come_here(self):
        self.print(f"Coming to beckoner")

    def action_sleep(self):
        self.state = False

    def action_pause(self):
        self.print(f"Pausing...")

    def action_resume(self):
        self.print("Resuming...")

    def print(self, message):
        print(message)

    def awake(self):
        return self.state
    
    def wake_up(self):
        self.state=True


if __name__ == "__main__":
    vc = None
    try:
        vc = VoiceController()
        vc.run()
    finally:
        if vc is not None:
            vc.rasa_controller.terminate()
            vc.action_controller.terminate()
