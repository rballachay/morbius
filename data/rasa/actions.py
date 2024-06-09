from typing import Text, List, Any, Dict

from rasa_sdk import Tracker, FormValidationAction, Action
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import EventType, SlotSet
from rasa_sdk.types import DomainDict


class ValidateRetrieveRoom(FormValidationAction):
    """
    This action simply tries to remind the form to continue 
    until we have extracted a room that is in our list of acceptable
    room.
    """
    def name(self) -> Text:
        return "validate_retrieve_room"
    
    # override super class
    async def get_extraction_events(
        self,
        dispatcher: "CollectingDispatcher",
        tracker: "Tracker",
        domain: "DomainDict",
    ) -> List[EventType]:
        custom_slots = {}
        slots_to_extract = await self.required_slots(
            self.domain_slots(domain), dispatcher, tracker, domain
        )

        for slot in slots_to_extract:
            extraction_output = await self._extract_slot(
                slot, dispatcher, tracker, domain
            )
                
            custom_slots.update(extraction_output)
            tracker.slots.update(extraction_output)

        if 'room' in tracker.slots.keys():
            if tracker.slots['room'] is None:
                dispatcher.utter_message(
                    text="Where would you like me to go? Say something like... \"Go to the lab\""
                )
        
        return [SlotSet(slot, value) for slot, value in custom_slots.items()]
            

class ValidateRetrieveObject(FormValidationAction):
    """
    This action simply tries to remind the form to continue 
    until we have extracted a room that is in our list of acceptable
    objects.
    """
    def name(self) -> Text:
        return "validate_retrieve_object"

    # override super class
    async def get_extraction_events(
        self,
        dispatcher: "CollectingDispatcher",
        tracker: "Tracker",
        domain: "DomainDict",
    ) -> List[EventType]:
        custom_slots = {}

        slots_to_extract = await self.required_slots(
            self.domain_slots(domain), dispatcher, tracker, domain
        )

        for slot in slots_to_extract:
            extraction_output = await self._extract_slot(
                slot, dispatcher, tracker, domain
            )
                
            custom_slots.update(extraction_output)
            tracker.slots.update(extraction_output)

            
        if 'object' in tracker.slots.keys():
            if tracker.slots['object'] is None:
                dispatcher.utter_message(
                    text="What do you want me to get? Say something like... \"Bring me a stethescope\""
                )
        
        return [SlotSet(slot, value) for slot, value in custom_slots.items()]
            

class ValidateRetrieveObjectFromLocation(FormValidationAction):
    """
    This will try to ensure we want to go to a location to retrieve 
    an object.
    """
    quote="\"Bring me a {object} from the {room}\""

    def name(self) -> Text:
        return "validate_retrieve_object_in_location"

    # override super class
    async def get_extraction_events(
        self,
        dispatcher: "CollectingDispatcher",
        tracker: "Tracker",
        domain: "DomainDict",
    ) -> List[EventType]:
        custom_slots = {}

        slots_to_extract = await self.required_slots(
            self.domain_slots(domain), dispatcher, tracker, domain
        )

        for slot in slots_to_extract:
            extraction_output = await self._extract_slot(
                slot, dispatcher, tracker, domain
            )
                
            custom_slots.update(extraction_output)
            tracker.slots.update(extraction_output)

        _object = None
        if 'object' in tracker.slots.keys():
            _object = tracker.slots['object']

        _room = None
        if 'room' in tracker.slots.keys():
            _room = tracker.slots['room']

    
        if (_object is None) and (_room is None):
            dispatcher.utter_message(
                text=f"What do you want me to get, and from what room?\
                    Say something like... {self.quote.format(object='scalpel',room='lab')}"
            )
        elif (_room is None):
            dispatcher.utter_message(
                text=f"What room do I get {_object} from?\
                    Say something like... {self.quote.format(object=_object,room='lab')}"
            )
        elif (_object is None):
            dispatcher.utter_message(
                text=f"What object do you want from {_room}?\
                    Say something like... {self.quote.format(object='scalpel',room=_room)}"
            )

        return [SlotSet(slot, value) for slot, value in custom_slots.items()]
    
class ActionMoveRobot(Action):
    """
    This is a custom action that should communicate with our robot, 
    although I don't know how that will work.
    """
    def name(self) -> Text:
        return "action_move_robot"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        # need to figure out the logic of how we are going to control
        # calls to the robot
        return []