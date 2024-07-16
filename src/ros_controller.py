import requests
import json
from config import ROBOT_ID

endpoint = "http://localhost:8080/{}"
headers = {'content-type': 'application/json'}

def make_body(id, param):
    return {"id": id, "param": param}

            
class RosControllerV1:
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

class RosControllerv2:
    """For the second version, going to use it to 
    send commands to the robot, using the ros.
    """
    def __init__(self):
        self.state = True

    def action_move_forward(self, distance):
        self.print(f"Moving forward {distance}...")
        response = requests.post(endpoint.format("forward"),
                                    data=json.dumps(make_body(ROBOT_ID, distance)), headers=headers)
        
        if response.status_code == 200:
            return "Robot confirmed!"
        else:
            return "Robot failed to move"
    
    def action_move_backward(self):
        self.print("Moving backward...")
    
    def action_turn_right(self, angle):
        self.print(f"Turning to the right {angle}...")
    
    def action_turn_left(self, angle):
        self.print(f"Turning to the left {angle}...")

    def action_stop(self):
        self.print(f"Stopping...")

    def action_resume(self):
        self.print("Resuming...")

    def action_sleep(self):
        self.state = False

    def print(self, message):
        print(message)

    def awake(self):
        return self.state
    
    def wake_up(self):
        self.state=True