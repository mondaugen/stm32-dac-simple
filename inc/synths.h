#ifndef SYNTHS_H
#define SYNTHS_H 
/* Some synths for testing sound on the STM32F4, fun I know. */
#include <stdlib.h>

/* Sampling rate, right now cannot change. */
#define SYNTHS_SR 44100

/* Size of cosine wavetable */
#define SYNTHS_COS_WT_SIZE 4096 

typedef struct {
    double amp;
    double freq;
    double phase;
} WaveParams;

typedef struct {
    size_t size;
    WaveParams waves[];
} WaveArray;

typedef struct __WaveArraySynth {
    double (*waveArrayTick)(struct __WaveArraySynth *);
    WaveArray *waveArray;
} WaveArraySynth;

WaveArray *WaveArray_new(size_t size);
double sineTone(double *phase, double freq, double sr);
double squareTone(double *phase, double freq, double sr);
double sawTone(double *phase, double freq, double sr);
void WaveArraySynth_init_sines(WaveArraySynth *synth, WaveArray *waveArray);

#endif /* SYNTHS_H */
