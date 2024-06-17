# morbius
Code for Robot Vision + Language Project w Dr. Joseph Vybihal

NOTE! REMEMBER that you need to re-name the submodule StyleTTS2/utils.py to StyleTTS2/munch_utils.py to avoid a name conflict with utils elsewhere.

## Development Notes

Whisper.cpp has an api already built. I think the best idea would be to make that as its own api in a docker container, maybe we can put our rasa api in there as well. Since we need the voice recording to be compiled, it would probably be best if this was in that same container. We could then have a separate python application that controls everything and calls all these separate apis and holds the logic to convert it into a command. 

This is necessary because the start-up time of these models is muuuch slower than the inference time, so if we start everything concurrently when the system is being initialized, we will have much lower total inference time accross the different modules.

## Help me!

### Segmentation fault, FasterWhisper

One problem that appeared multiple times during development is a Segmentation fault caused by having multiple copies of the shared library `libiomp5.dylib`, one in ctranslate2 and the other in torch:

```
Segmentation fault: 11
/Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/multiprocessing/resource_tracker.py:224: UserWarning: resource_tracker: There appear to be 1 leaked semaphore objects to clean up at shutdown
```

To fix this problem, find the copies of the shared library inside of your enviornment and delete those associated with torch, then symlink those to the ctranslate2 version:

```
find /Users/RileyBallachay/opt/anaconda3/envs/python3.10 -name "libiomp5.dylib"

rm /Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/site-packages/torch/lib/libiomp5.dylib
rm /Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/site-packages/functorch/.dylibs/libiomp5.dylib

ln -s /Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/site-packages/ctranslate2/.dylibs/libiomp5.dylib /Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/site-packages/functorch/.dylibs/libiomp5.dylib

ln -s /Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/site-packages/ctranslate2/.dylibs/libiomp5.dylib /Users/RileyBallachay/opt/anaconda3/envs/python3.10/lib/python3.10/site-packages/torch/lib/libiomp5.dylib
```