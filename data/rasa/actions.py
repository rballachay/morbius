from typing import Text, List, Any, Dict

from rasa_sdk import Tracker, FormValidationAction
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

        if 'room' not in  custom_slots.keys():
            dispatcher.utter_message(
                text="Where would you like me to go? Say something like... \"Go to the lab\""
                )
        
        return [SlotSet(slot, value) for slot, value in custom_slots.items()]
            
    