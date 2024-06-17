import sys
import inspect
from typing import Text, List, Any, Dict

from rasa_sdk import Tracker, Action
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet


class ActionMoveForward(Action):
    def name(self) -> Text:
        return "action_move_forward"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:

        distance = tracker.slots.get("distance", None)
        dispatcher.utter_message(f"Moving forward {distance if distance else ''}")
        dispatcher.utter_custom_json({"action": self.name(), "distance": distance})
        return [SlotSet('distance',None)]


class ActionMoveBackward(Action):
    def name(self) -> Text:
        return "action_move_backward"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(f"Moving backward")
        dispatcher.utter_custom_json({"action": self.name()})
        return []


class ActionTurnRight(Action):

    def name(self) -> Text:
        return "action_turn_right"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        angle = tracker.slots.get("angle", None)
        dispatcher.utter_message(f"Turning right {angle if angle else ''}")
        dispatcher.utter_custom_json({"action": self.name(), "angle":angle})
        return [SlotSet('angle',None)]

class ActionTurnLeft(Action):

    def name(self) -> Text:
        return "action_turn_left"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        angle = tracker.slots.get("angle", None)
        dispatcher.utter_message(f"Turning left {angle if angle else ''}")
        dispatcher.utter_custom_json({"action": self.name(), "angle":angle})
        return [SlotSet("angle", None)]

class ActionSleep(Action):
    """
    This action is to sleep
    """

    def name(self) -> Text:
        return "action_sleep"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(f"Going to sleep...")
        dispatcher.utter_custom_json({"action": self.name()})
        return []


class ActionStop(Action):
    """
    This action is to pause
    """

    def name(self) -> Text:
        return "action_stop"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(f"Stopping ongoing task...")
        dispatcher.utter_custom_json({"action": self.name()})
        return []


class ActionResume(Action):
    """
    This action is to resume
    """

    def name(self) -> Text:
        return "action_resume"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(f"Resuming...")
        dispatcher.utter_custom_json({"action": self.name()})
        return []

class ActionListCommands(Action):
    def name(self) -> Text:
        return "action_list_commands"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(f"")
        
        current_module = sys.modules[__name__]
    
        # Get all class names defined in the current module
        class_names = [obj().name() for name, obj in globals().items() if inspect.isclass(obj) and obj.__module__ == current_module.__name__]
        class_names.remove(self.name())
        class_names = [' '.join(i.split('_')) for i in class_names]
        class_names = [i.replace('action ','') for i in class_names]
        dispatcher.utter_message(f"Commands I am able to accept include: {', '.join(class_names)}")
        return []