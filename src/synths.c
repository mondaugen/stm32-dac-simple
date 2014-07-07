#include "synths.h"
#include <string.h> 
#include <math.h> 

static double cosWaveTable[SYNTHS_COS_WT_SIZE];

/* generates a sine tone at a specific frequency and sample rate */
double sineTone(double *phase, double freq, double sr)
{
    *phase += freq / sr;
    while (*phase > 1.) { *phase -= 1.; }
    return sin(2 * M_PI * (*phase));
}

double squareTone(double *phase, double freq, double sr)
{
    *phase += freq / sr;
    while (*phase > 1.) { *phase -= 1.; }
    if (*phase > 0.5) { return -1; };
    return 1;
}

double sawTone(double *phase, double freq, double sr)
{
    *phase += freq / sr;
    while (*phase > 1.) { *phase -= 1.; }
    return (*phase - 0.5) * 2;
}

/* generates cosine tone looked up from the wavetable */

static double WaveArray_sine_tick(WaveArraySynth *waveArraySynth)
{
    double result = 0;
    size_t i;
    WaveArray *wa = waveArraySynth->waveArray;
    for (i = 0; i < wa->size; i++) {
        result += sineTone(&(wa->waves[i].phase),
                        wa->waves[i].freq,
                        SYNTHS_SR) * wa->waves[i].amp;
    }
    return result;
}

static double WaveArray_coswt_tick(WaveArraySynth *waveArraySynth)
{
    double result = 0;
    size_t i;
    WaveArray *wa = waveArraySynth->waveArray;
    for (i = 0; i < wa->size; i++) {
        result += sineTone(&(wa->waves[i].phase),
                        wa->waves[i].freq,
                        SYNTHS_SR) * wa->waves[i].amp;
    }
    return result;
}


void WaveArraySynth_init_sines(WaveArraySynth *synth, WaveArray *waveArray)
{
    synth->waveArray = waveArray;
    synth->waveArrayTick = &WaveArray_sine_tick;
}

WaveArray *WaveArray_new(size_t size)
{

    WaveArray *result = (WaveArray*)malloc(sizeof(WaveArray)
            + sizeof(WaveParams) * size);
    memset(result, 0, sizeof(WaveArray) + sizeof(WaveParams) * size);
    result->size = size;
    return result;
}
