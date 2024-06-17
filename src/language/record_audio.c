#include <stdio.h>
#include <stdlib.h>
#include <portaudio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <fvad.h>

// apparently the internal bitrate of vfad is 8000, so it will be downsampled 
// when running that library, but this is 16 kHz for our recording at least
// https://github.com/dpirch/libfvad/blob/master/include/fvad.h#L70
//#define SAMPLE_RATE 16000

// this has to be a multiple of 80, 160 or 240
// https://github.com/dpirch/libfvad/blob/master/include/fvad.h#L82
#define FRAMES_PER_BUFFER 160
#define NUM_CHANNELS 1

typedef struct {
    float *recordedSamples;
    int frameIndex;
    int maxFrameIndex;
    int silenceFrameCount;
    int silenceDuration;
    int sampleRate;
    int vocalFrames;
    Fvad *vad;
} paData;

static int recordCallback(const void *inputBuffer, void *outputBuffer,
                          unsigned long framesPerBuffer,
                          const PaStreamCallbackTimeInfo *timeInfo,
                          PaStreamCallbackFlags statusFlags,
                          void *userData) {
    paData *data = (paData *) userData;
    const float *rptr = (const float *) inputBuffer;
    float *wptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
    long framesToCalc;
    long i;
    int finished;
    int silentFrames = 0;
    int vadres = -1;
    int16_t *intBuf = NULL;

    if (data->frameIndex + framesPerBuffer > data->maxFrameIndex) {
        framesToCalc = data->maxFrameIndex - data->frameIndex;
        finished = paComplete;
    } else {
        framesToCalc = framesPerBuffer;
        finished = paContinue;
    }

    if (inputBuffer == NULL) {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = 0.0f;  // If no input, insert silence
        }
    } else {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = *rptr++;
        }

    }

    if (framesToCalc > SIZE_MAX / sizeof (float)
            || !(intBuf = malloc(framesToCalc * sizeof *intBuf))) {
        fprintf(stderr, "failed to allocate buffers\n");
        return -1;
    }

    for (int i = 0; i < framesToCalc; i++){
        intBuf[i] = data->recordedSamples[data->frameIndex+i] * INT16_MAX;
    }

    // this returns for n number of frames if the speaker is talking or not.
    // we know this corresponds to 160/16000 s, so we can see if the speaker
    // is done talking or not
    vadres = fvad_process(data->vad, intBuf, framesToCalc);
    free(intBuf); 

    if (vadres==0){
        silentFrames+=framesToCalc;
    }else{
        data->silenceFrameCount=0;
        data->vocalFrames+=1;
    }

    data->frameIndex += framesToCalc;

    // if we have had 10 frames of voice, we can start adding silence
    if (data->vocalFrames > 10){
        data->silenceFrameCount += silentFrames;
    }

    if (data->silenceFrameCount >= data->silenceDuration * data->sampleRate) {
        printf("Silence detected.\n");
        finished = paComplete; 
    }

    return finished;
}

int record(const int sampleRate, float *array, const int nSeconds, 
            const int silenceDuration, const int vadMode) {

    Fvad *vad = NULL;
    PaError err = paNoError;
    PaStream *stream;
    paData data;
    int numSamples = nSeconds * sampleRate;
    data.recordedSamples = array;
    data.frameIndex = 0;
    data.maxFrameIndex = numSamples;
    data.silenceFrameCount = 0;
    data.silenceDuration=silenceDuration;
    data.sampleRate=sampleRate;
    data.vocalFrames = 0;

    vad = fvad_new();
    if (!vad) {
        fprintf(stderr, "out of memory\n");
        return -1;
    }
    fvad_set_mode(vad, vadMode);
    fvad_set_sample_rate(vad, FRAMES_PER_BUFFER);
    data.vad = vad;

    if (data.recordedSamples == NULL) {
        printf("Could not allocate memory for audio data.\n");
        return -1;
    }

    err = Pa_Initialize();
    if (err != paNoError) goto done;

    err = Pa_OpenDefaultStream(&stream, NUM_CHANNELS, 0, paFloat32,
                               sampleRate, FRAMES_PER_BUFFER,
                               recordCallback, &data);
    if (err != paNoError) goto done;

    err = Pa_StartStream(stream);
    if (err != paNoError) goto done;

    printf("Recording for %d seconds.\n", nSeconds);
    while (Pa_IsStreamActive(stream)) {
        Pa_Sleep(100);  // Sleep for a short duration
    }

    err = Pa_StopStream(stream);
    if (err != paNoError) goto done;

    err = Pa_CloseStream(stream);
    if (err != paNoError) goto done;

done:
    Pa_Terminate();
    if (err != paNoError) {
        fprintf(stderr, "An error occurred: %s\n", Pa_GetErrorText(err));
        return -1;
    }
    return 0;
}