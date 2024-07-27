# Language Model

This is the language model. There are three main parts: 

1. Audio recording and voice activity detection - C
2. Speech to text - uses whisper (openai) model - Python
3. Text to speech - uses nixtts, can use a variety of models - Python

The first part does not require a machine learning model, but the second two do.

### Voice activity detection

The first part of the voice module is recording the speaker. We may be able to use active listening to that the app is always listening and able to be awakened, like siri. Once that is done, we need to know when the speaker is done talking, so that we may start running our dialogue system. For this, we are using one of the most popular libraries, [VAD from WebRTC](https://webrtc.org/). This package was installed using the [libfvad repo on github](https://github.com/dpirch/libfvad). Follow these steps for installation:

```bash
# ensure you are in the base directory /morbius
git submodule --init update submodules/libfvad
cd submodules/libfvad
[[ $OSTYPE == 'darwin'* ]] && brew install autoconf && brew install automake \
    || apt install autoconf libtool pkg-config 
make
make install
```

You can then build the record_audio shared library by coming to this directory and running:

```bash
[[ $OSTYPE == 'darwin'* ]] && brew portaudio || apt install portaudio
gcc -shared -o record_audio.so -fPIC record_audio.c -lportaudio -lfvad
```