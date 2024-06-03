## Development Notes

### Whisper.cpp

__Notes on real-time transcription__: Whisper is not developed to transcribe in real-time. It can be heavy, and would require a buffering system. For more information on the subject, [follow this link](https://github.com/ggerganov/whisper.cpp/issues/1653#issuecomment-1862038088). The system was therefore set up to first record audio, determine when the speaker was done speaking, r