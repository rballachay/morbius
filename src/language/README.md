## Development Notes

### Voice activity detection

The first part of the voice module is recording the speaker. We may be able to use active listening to that the app is always listening and able to be awakened, like siri. Once that is done, we need to know when the speaker is done talking, so that we may start running our dialogue system. For this, we are using one of the most popular libraries, [VAD from WebRTC](https://webrtc.org/). This package was installed using the [libfvad repo on github](https://github.com/dpirch/libfvad). Follow these steps for installation:

```bash
git clone https://github.com/dpirch/libfvad.git
cd libfvad
[[ $OSTYPE == 'darwin'* ]] && brew install autoconf && brew install automake \
    || apt install autoconf libtool pkg-config 
make
make install
```

You can then build the record_audio shared library by coming to this directory and running:


```bash
[[ $OSTYPE == 'darwin'* ]] && brew portaudio || apt install portaudio
gcc -shared -o record_audio.so -fPIC record_audio.c -lportaudio
```

### Whisper.cpp

__Notes on real-time transcription__: Whisper is not developed to transcribe in real-time. It can be heavy, and would require a buffering system. For more information on the subject, [follow this link](https://github.com/ggerganov/whisper.cpp/issues/1653#issuecomment-1862038088). The system was therefore set up to first record audio, determine when the speaker was done speaking, save this to a file, then pass this file to whsiper. This also allows us to persist data so that it can be re-run if the API is busy or things need to be performed async.
