# morbius
Code for Robot Vision + Language Project w Dr. Joseph Vybihal

## Setup on raspberry pi

To set up on raspberry pi, first follow the steps outlined in `rasp/README.md`. Once that is done, you can install all the dependencies and compile as follows:


```bash 
bash rasp/setup.sh --miniconda --audio --vision --slam
```

This can take anywhere from 30 mintues to 2 hours, so ensure you are patient. If you want to install just one of these, just use one of the flags. However, note that miniconda needs to be installed in order to install the audio system!!!

## Running the different components 

### Language

The language daemon can be run by first running: `bash rasp/setup.sh --miniconda --audio`, then navigating to `language` and running `python3 daemon.py`. This will create an open session that listens to the microphone until the user says "Hey robot", at which point the full voice model will activate and ask for responses from the user. These responses include "move forward <some distance>", "move backward", "stop", "resume", "sleep", "turn right <some angle>", "turn left <some angle>" and "can you please help me?". The robot will continue asking for input until it is asked to sleep, at which point it will return to listening for "Hey robot".


### Vision

The vision models can be run by first running `bash rasp/setup.sh --vision --slam`, to build the components, then running the corresponding commands shown in `vision/README.md`.

