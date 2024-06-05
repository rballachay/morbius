# morbius
Code for Robot Vision + Language Project w Dr. Joseph Vybihal


## Development Notes

Whisper.cpp has an api already built. I think the best idea would be to make that as its own api in a docker container, maybe we can put our rasa api in there as well. Since we need the voice recording to be compiled, it would probably be best if this was in that same container. We could then have a separate python application that controls everything and calls all these separate apis and holds the logic to convert it into a command. 

This is necessary because the start-up time of these models is muuuch slower than the inference time, so if we start everything concurrently when the system is being initialized, we will have much lower total inference time accross the different modules.