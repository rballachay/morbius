#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <espeak-ng/speak_lib.h>

espeak_AUDIO_OUTPUT output = AUDIO_OUTPUT_RETRIEVAL;
char *path = NULL;
void* user_data;
unsigned int *identifier;

typedef struct {
    short *data;
    size_t size;
    size_t capacity;
} AudioBuffer;

AudioBuffer audioBuffer = {NULL, 0, 0};

// Callback function to handle the raw audio data
int SynthCallback(short *wav, int num_samples, espeak_EVENT *events) {
    if (wav == NULL) {
        // No more samples, synthesis is done
        return 1;
    }

    // Ensure the buffer has enough capacity
    if (audioBuffer.size + num_samples > audioBuffer.capacity) {
        audioBuffer.capacity = (audioBuffer.size + num_samples) * 2;
        audioBuffer.data = realloc(audioBuffer.data, audioBuffer.capacity * sizeof(short));
        if (audioBuffer.data == NULL) {
            fprintf(stderr, "Failed to allocate memory\n");
            exit(EXIT_FAILURE);
        }
    }

    // Copy the new samples into the buffer
    memcpy(audioBuffer.data + audioBuffer.size, wav, num_samples * sizeof(short));
    audioBuffer.size += num_samples;

    return 0;
}

short* say_text(const char* text, size_t *output_size) {
    int buflength = 500, options = 0;
    unsigned int position = 0, position_type = 0, end_position = 0, flags = espeakCHARS_AUTO;

    // Initialize eSpeak with AUDIO_OUTPUT_RETRIEVAL to get raw audio data
    espeak_Initialize(output, buflength, path, options);

    espeak_VOICE voice;
    memset(&voice, 0, sizeof(espeak_VOICE)); // Zero out the voice first
    const char *langNativeString = "en"; // Set voice by properties
    voice.languages = langNativeString;
    voice.name = "US";
    voice.variant = 1;
    voice.gender = 2;
    espeak_SetVoiceByProperties(&voice);

    // Initialize the audio buffer
    audioBuffer.data = NULL;
    audioBuffer.size = 0;
    audioBuffer.capacity = 0;

    // Set the synthesis callback
    espeak_SetSynthCallback(SynthCallback);

    //printf("Saying '%s'...\n", text);
    espeak_Synth(text, buflength, position, position_type, end_position, flags, identifier, user_data);
    espeak_Synchronize(); // Wait for the speech to finish
    //printf("Done\n");

    // Return the accumulated audio data
    *output_size = audioBuffer.size;
    return audioBuffer.data;
}

int main() {
    const char *text = "Hello, this is a test.";
    size_t output_size;
    short *audio_data = say_text(text, &output_size);

    if (audio_data != NULL) {
        printf("Received %zu samples\n", output_size);

        // Do something with the audio data
        // For example, write it to a file or play it back

        // Free the allocated audio data
        free(audio_data);
    }

    return 0;
}
