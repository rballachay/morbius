import requests
import json
import re
from config import ROBOT_ID, DEFAULT_ANGLE, DEFAULT_FORWARD, DEFAULT_BACKWARD

endpoint = "http://localhost:8080/{}"
headers = {'content-type': 'application/json'}

def make_body(id, param):
    return {"id": id, "param": param}

def convert_number_units(number):
    """This is used to convert a variety of formats of string and 
    number-format numbers into true floats, all with a consistent 
    unit of either cm or degrees. Here are some samples:

    'fifty degrees' -> 50.0
    'a hundred fifty centimetres' -> 150.0
    '2 yards' -> 182.88
    'twenty' -> 20.0
    'seventy eight degrees' -> 78.0
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
    }
    units = {
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
    words = re.split(r'[\s-]+', number.lower())

    current_number = 0
    for word in words:
        try:
            float_word = float(word)
            current_number += float_word
            continue
        except:
            pass

        if word in words_to_numbers:
            current_number += words_to_numbers[word]
        elif word in units:
            if current_number==0:
                current_number=1
            # break once we are at units
            current_number *= units[word]
            break
    return current_number


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
        if distance is None:
            distance = DEFAULT_FORWARD

        distance_cm = convert_number_units(distance)
        self.print(f"Moving forward {distance_cm} cm...")
        
        try:
            response = requests.post(endpoint.format("forward"),
                                        data=json.dumps(make_body(ROBOT_ID, distance_cm)), headers=headers)        
        except requests.exceptions.ConnectionError:
            return "Failed to communicate with robot"
        
        print(response.text)

        if response.status_code == 200:
            return "Robot moved forward"
        else:
            return "Robot failed to move"
    
    def action_move_backward(self, distance):
        if distance is None:
            distance = DEFAULT_BACKWARD

        distance_cm = convert_number_units(distance)

        self.print(f"Moving backward {distance_cm} cm...")

        try:
            response = requests.post(endpoint.format("backward"),
                                        data=json.dumps(make_body(ROBOT_ID, distance_cm)), headers=headers)
        except requests.exceptions.ConnectionError:
            return "Failed to communicate with robot"
        
        print(response.text)

        if response.status_code == 200:
            return "Robot moved backward"
        else:
            return "Robot failed to move"
    
    def action_turn_right(self, angle):
        if angle is None:
            angle=DEFAULT_ANGLE
        self.print(f"Turning right {angle}...")  
        angle = convert_number_units(angle)

        try:
            response = requests.post(endpoint.format("turnright"),
                                        data=json.dumps(make_body(ROBOT_ID, angle)), headers=headers)
        except requests.exceptions.ConnectionError:
            return "Failed to communicate with robot"
        
        print(response.text)

        if response.status_code == 200:
            return "Robot confirmed!"
        else:
            return "Robot failed to move"
    
    def action_turn_left(self, angle):
        if angle is None:
            angle = DEFAULT_ANGLE
        self.print(f"Turning left {angle}...")  
        angle = convert_number_units(angle)

        try:
            response = requests.post(endpoint.format("turnleft"),
                                        data=json.dumps(make_body(ROBOT_ID, angle)), headers=headers)
        except requests.exceptions.ConnectionError:
            return "Failed to communicate with robot"
        
        print(response.text)
        
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