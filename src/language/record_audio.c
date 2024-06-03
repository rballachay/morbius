#include <stdio.h>
#include <stdlib.h>
#include <portaudio.h>
#include <math.h>

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 512
#define NUM_CHANNELS 1
#define NUM_SECONDS 10
#define SILENCE_THRESHOLD 0.1f  // Adjust as needed
#define SILENCE_DURATION  2      // Duration in seconds

typedef struct {
    float *recordedSamples;
    int frameIndex;
    int maxFrameIndex;
    int silenceFrameCount;
    float maxAmplitude;
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

            // Update maximum amplitude
            if (fabs(*rptr) > data->maxAmplitude) {
                data->maxAmplitude = fabs(*rptr);
                data->silenceFrameCount=0; // reset silence count if new theshold

            }

            if (fabs(rptr[i]) < data->maxAmplitude * SILENCE_THRESHOLD) {
                silentFrames++;
            }
        }
    }

    data->frameIndex += framesToCalc;

    // Check for silence
    data->silenceFrameCount += silentFrames;
    if (data->silenceFrameCount >= SILENCE_DURATION * SAMPLE_RATE) {
        printf("Silence detected.\n");
        finished = paComplete; 
    }

    return finished;
}

int detect_silence(float *samples, int numSamples, int sampleRate) {
    int silentFrames = 0;
    int silentThreshold = sampleRate * SILENCE_DURATION;

    for (int i = 0; i < numSamples; i++) {
        if (fabs(samples[i]) < SILENCE_THRESHOLD) {
            silentFrames++;
            if (silentFrames >= silentThreshold) {
                return 1;  // Silence detected
            }
        } else {
            silentFrames = 0;  // Reset counter if noise detected
        }
    }
    return 0;  // No silence detected
}

int record(const char *filename) {
    PaError err = paNoError;
    PaStream *stream;
    paData data;
    int numSamples = NUM_SECONDS * SAMPLE_RATE;
    int numBytes = numSamples * sizeof(float);
    data.recordedSamples = (float *) malloc(numBytes);
    data.frameIndex = 0;
    data.maxFrameIndex = numSamples;
    data.silenceFrameCount = 0;
    data.maxAmplitude = 0.0f;

    if (data.recordedSamples == NULL) {
        printf("Could not allocate memory for audio data.\n");
        return -1;
    }

    err = Pa_Initialize();
    if (err != paNoError) goto done;

    err = Pa_OpenDefaultStream(&stream, NUM_CHANNELS, 0, paFloat32,
                               SAMPLE_RATE, FRAMES_PER_BUFFER,
                               recordCallback, &data);
    if (err != paNoError) goto done;

    err = Pa_StartStream(stream);
    if (err != paNoError) goto done;

    printf("Recording for %d seconds.\n", NUM_SECONDS);
    while (Pa_IsStreamActive(stream)) {
        Pa_Sleep(100);  // Sleep for a short duration
    }

    err = Pa_StopStream(stream);
    if (err != paNoError) goto done;

    err = Pa_CloseStream(stream);
    if (err != paNoError) goto done;

    // Save recorded audio to file
    FILE *file = fopen(filename, "wb");
    if (!file) {
        printf("Error: could not open file for writing.\n");
        goto done;
    }
    fwrite(data.recordedSamples, sizeof(float), numSamples, file);
    fclose(file);
    printf("Recording saved to %s.\n", filename);
done:
    Pa_Terminate();
    if (data.recordedSamples) free(data.recordedSamples);
    if (err != paNoError) {
        fprintf(stderr, "An error occurred: %s\n", Pa_GetErrorText(err));
        return -1;
    }
    return 0;
}