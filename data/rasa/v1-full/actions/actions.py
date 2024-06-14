from typing import Text, List, Any, Dict

from rasa_sdk import Tracker, Action
from rasa_sdk.executor import CollectingDispatcher


class ActionMoveRobot(Action):
    """
    This action is for moving a robot to a certain destination. It
    takes only the room as a parameter.
    """

    def name(self) -> Text:
        return "action_move_robot"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:

        room = tracker.slots.get("room", None)
        dispatcher.utter_custom_json({"action": self.name(), "room": room})
        return []


class ActionFindObject(Action):
    """
    This action is for finding a particular object. It can also
    be called in conjunction with the move_robot
    """

    def name(self) -> Text:
        return "action_find_object"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:

        object = tracker.slots.get("object", None)
        dispatcher.utter_custom_json({"action": self.name(), "object": object})
        return []


class ActionComeHere(Action):
    """
    This action is to move robot to beckoner
    """

    def name(self) -> Text:
        return "action_come_here"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
        dispatcher.utter_custom_json({"action": self.name()})
        return []


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
        dispatcher.utter_custom_json({"action": self.name()})
        return []


class ActionPause(Action):
    """
    This action is to pause
    """

    def name(self) -> Text:
        return "action_pause"

    def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict[Text, Any]]:
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
        dispatcher.utter_custom_json({"action": self.name()})
        return []
