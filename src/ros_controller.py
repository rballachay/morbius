import requests
import json
from config import ROBOT_ID
import re

endpoint = "http://localhost:8080/{}"
headers = {'content-type': 'application/json'}

def make_body(id, param):
    return {"id": id, "param": param}


def convert_number_units(number):
    """convert a distance or angle into a float
    """
    # Dictionary to map words to numeric values
    words_to_numbers = {
        'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4,
        'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9,
        'ten': 10, 'eleven': 11, 'twelve': 12, 'thirteen': 13, 'fourteen': 14,
        'fifteen': 15, 'sixteen': 16, 'seventeen': 17, 'eighteen': 18, 'nineteen': 19,
        'twenty': 20, 'thirty': 30, 'forty': 40, 'fifty': 50,
        'sixty': 60, 'seventy': 70, 'eighty': 80, 'ninety': 90,
        'hundred': 100, 'thousand': 1000, 'million': 1000000,
        'centimeter': 1, 'centimeters': 1, 'cm': 1,
        'meter': 100, 'meters': 100, 'metre': 100, 'metres': 100, 'm': 100,
        'millimeter': 0.1, 'millimeters': 0.1, 'mm': 0.1,
        'inch': 2.54, 'inches': 2.54, 'in': 2.54,
        'foot': 30.48, 'feet': 30.48, 'ft': 30.48,
        'yard': 91.44, 'yards': 91.44, 'yd': 91.44,

        # this will also work for an angle
        "degree":1, "degrees":1
    }

    # Split the distance string into words
    words = number.lower().split()

    total_cm = 0
    current_number = 0
    last_multiplier = 1

    for word in words:
        if word in words_to_numbers:
            if words_to_numbers[word] >= 100:
                if current_number == 0:
                    current_number = 1
                current_number *= words_to_numbers[word]
                last_multiplier = words_to_numbers[word]
            else:
                current_number += words_to_numbers[word] * last_multiplier
                if words_to_numbers[word] >= 10 and words_to_numbers[word] < 100:
                    last_multiplier = words_to_numbers[word]

    total_cm = current_number
    return total_cm


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
        distance_cm = convert_number_units(distance)
        self.print(f"Moving forward {distance_cm} cm...")
        response = requests.post(endpoint.format("forward"),
                                    data=json.dumps(make_body(ROBOT_ID, distance_cm)), headers=headers)
        
        print(response.text)
        if response.status_code == 200:
            return "Robot moved forward"
        else:
            return "Robot failed to move"
    
    def action_move_backward(self):
        self.print(f"Moving backward {50} cm...")
        response = requests.post(endpoint.format("forward"),
                                    data=json.dumps(make_body(ROBOT_ID, 50)), headers=headers)
        if response.status_code == 200:
            return "Robot moved backward"
        else:
            return "Robot failed to move"
    
    def action_turn_right(self, angle):
        self.print(f"Turning right {angle}...")  
        angle = convert_number_units(angle)
        response = requests.post(endpoint.format("turnright"),
                                     data=json.dumps(make_body(ROBOT_ID, angle)), headers=headers)

        if response.status_code == 200:
            return "Robot confirmed!"
        else:
            return "Robot failed to move"
    
    def action_turn_left(self, angle):
        self.print(f"Turning left {angle}...")  
        angle = convert_number_units(angle)
        response = requests.post(endpoint.format("turnleft"),
                                     data=json.dumps(make_body(ROBOT_ID, angle)), headers=headers)

        if response.status_code == 200:
            return "Robot confirmed!"
        else:
            return "Robot failed to move"

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