# Text to speech

We tested out a variety of text-to-speech models, and discovered that nix-tts gave the best performance and latency. With that said, you can still use any of the different models by changing the selected model in the `config.py` file. The default, and only model that will work without separately downloading the weights is [NixTTS](https://github.com/rendchevi/nix-tts). This uses the model `nix-ljspeech-deterministic-v0.1` which should be downloaded to the folder `models/` automatically from Riley's google drive using a public share-able link. If you are having problems downloading that model from the link, you may download it separately from the following source: [Pre-trained Models](https://drive.google.com/drive/folders/1GbFOnJsgKHCAXySm2sTluRRikc4TAWxJ). This will need to be un-zipped and placed in the `models/` folder.

## Pyaudio and channel problems

One commonly occurring problem is that pyaudio may not connect properly to the microphone. Assuming you're not using the Jabra microphone, you will need to edit lines 83-100 in the `text_to_speech.py` file, as this is where the channels are automatically set. 