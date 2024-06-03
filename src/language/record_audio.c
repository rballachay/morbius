#include <stdio.h>
#include <stdlib.h>
#include <portaudio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 512
#define NUM_CHANNELS 1

typedef struct {
    float *recordedSamples;
    int frameIndex;
    int maxFrameIndex;
    int silenceFrameCount;
    float maxAmplitude;
    float silenceThresh;
    int silenceDuration;
} paData;

#pragma pack(push, 1)
typedef struct {
    char chunkID[4];       // "RIFF"
    uint32_t chunkSize;    // Size of the entire file minus 8 bytes
    char format[4];        // "WAVE"
    char subchunk1ID[4];   // "fmt "
    uint32_t subchunk1Size;// 16 for PCM
    uint16_t audioFormat;  // PCM = 1
    uint16_t numChannels;  // Number of channels
    uint32_t sampleRate;   // Sample rate
    uint32_t byteRate;     // SampleRate * NumChannels * BitsPerSample/8
    uint16_t blockAlign;   // NumChannels * BitsPerSample/8
    uint16_t bitsPerSample;// Bits per sample
    char subchunk2ID[4];   // "data"
    uint32_t subchunk2Size;// Number of bytes in data
} WavHeader;
#pragma pack(pop)

void writeWavHeader(FILE *file, int sampleRate, int numChannels, int numSamples) {
    WavHeader header;
    
    // RIFF header
    memcpy(header.chunkID, "RIFF", 4);
    header.chunkSize = 36 + numSamples * numChannels * sizeof(int16_t);
    memcpy(header.format, "WAVE", 4);
    
    // fmt subchunk
    memcpy(header.subchunk1ID, "fmt ", 4);
    header.subchunk1Size = 16;
    header.audioFormat = 1; // PCM
    header.numChannels = numChannels;
    header.sampleRate = sampleRate;
    header.byteRate = sampleRate * numChannels * sizeof(int16_t);
    header.blockAlign = numChannels * sizeof(int16_t);
    header.bitsPerSample = 16;
    
    // data subchunk
    memcpy(header.subchunk2ID, "data", 4);
    header.subchunk2Size = numSamples * numChannels * sizeof(int16_t);
    
    fwrite(&header, sizeof(WavHeader), 1, file);
}

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

            if (fabs(rptr[i]) < data->maxAmplitude * data->silenceThresh) {
                silentFrames++;
            }
        }
    }

    data->frameIndex += framesToCalc;

    // Check for silence
    data->silenceFrameCount += silentFrames;
    if (data->silenceFrameCount >= data->silenceDuration * SAMPLE_RATE) {
        printf("Silence detected.\n");
        finished = paComplete; 
    }

    return finished;
}

int record(const char *filename, const int nSeconds, 
            const float silenceThresh, const int silenceDuration) {

    PaError err = paNoError;
    PaStream *stream;
    paData data;
    int numSamples = nSeconds * SAMPLE_RATE;
    int numBytes = numSamples * sizeof(float);
    data.recordedSamples = (float *) malloc(numBytes);
    data.frameIndex = 0;
    data.maxFrameIndex = numSamples;
    data.silenceFrameCount = 0;
    data.maxAmplitude = 0.0f;
    data.silenceThresh=silenceThresh;
    data.silenceDuration=silenceDuration;

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

    printf("Recording for %d seconds.\n", nSeconds);
    while (Pa_IsStreamActive(stream)) {
        Pa_Sleep(100);  // Sleep for a short duration
    }

    err = Pa_StopStream(stream);
    if (err != paNoError) goto done;

    err = Pa_CloseStream(stream);
    if (err != paNoError) goto done;

    // Save recorded audio to file
    int recordedSamples = data.frameIndex;
    FILE *file = fopen(filename, "wb");
    if (!file) {
        printf("Error: could not open file for writing.\n");
        goto done;
    }
    writeWavHeader(file, SAMPLE_RATE, NUM_CHANNELS, recordedSamples);
    
    // Convert float samples to int16_t samples
    int16_t *intSamples = (int16_t *) malloc(recordedSamples * sizeof(int16_t));
    for (int i = 0; i < recordedSamples; i++) {
        intSamples[i] = (int16_t) (data.recordedSamples[i] * 32767.0f);
    }
    fwrite(intSamples, sizeof(int16_t), recordedSamples, file);
    fclose(file);
    free(intSamples);
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