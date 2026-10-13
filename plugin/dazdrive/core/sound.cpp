// Feedback sounds (gearbox mode, ACC distance, feature on/off, AEB), played
// asynchronously from WAVs built in memory (winmm PlaySound; works under
// Proton). One sound at a time; a new one replaces the old.
//
// Two voices, chosen by what carries the meaning:
//  - pitch and direction (mode, on/off) use a soft bell
//  - counts and pulses (ACC distance, the AEB steps) use a dry, steady tone that is easy to count
//  - the AEB steps climb as one alarm: plain sine pips, then triangle pulses, then the square pulses
#include "suite.h"
#include <math.h>
#include <mmsystem.h>

float g_volume = 0.35f;

enum Wave { W_BELL, W_SOFT, W_GLIDE, W_TRI, W_SQ, W_PIP };
struct Note { Wave w; float at_s; int ms; float f0, f1; };
struct CueDef { float vol; int n; Note notes[8]; };

// Levels were tuned by ear at g_volume 0.35; other volumes scale from there.
static const CueDef CUES[CUE_COUNT] = {
    /* MODE_ECO    */ { 0.30f, 1, { { W_BELL, 0,    400, 392, 0 } } },
    /* MODE_NORMAL */ { 0.30f, 1, { { W_BELL, 0,    400, 523, 0 } } },
    /* MODE_POWER  */ { 0.30f, 2, { { W_BELL, 0,    350, 523, 0 }, { W_BELL, 0.12f, 450, 784, 0 } } },
    /* MODE_M      */ { 0.30f, 2, { { W_BELL, 0,    250, 659, 0 }, { W_BELL, 0.14f, 300, 659, 0 } } },
    /* ACC_1       */ { 0.25f, 1, { { W_SOFT, 0,     90, 660, 0 } } },
    /* ACC_2       */ { 0.25f, 2, { { W_SOFT, 0,     90, 660, 0 }, { W_SOFT, 0.14f, 90, 660, 0 } } },
    /* ACC_3       */ { 0.25f, 3, { { W_SOFT, 0,     90, 660, 0 }, { W_SOFT, 0.14f, 90, 660, 0 }, { W_SOFT, 0.28f, 90, 660, 0 } } },
    /* ACC_OFF     */ { 0.25f, 1, { { W_GLIDE, 0,   220, 660, 330 } } },
    /* ON          */ { 0.30f, 2, { { W_BELL, 0,    300, 523, 0 }, { W_BELL, 0.10f, 450, 784, 0 } } },
    /* OFF         */ { 0.30f, 2, { { W_BELL, 0,    300, 784, 0 }, { W_BELL, 0.10f, 450, 523, 0 } } },
    /* AEB_WARN    */ { 0.30f, 3, { { W_PIP, 0,    100, 900, 0 }, { W_PIP, 0.16f, 100, 900, 0 }, { W_PIP, 0.32f, 100, 900, 0 } } },
    /* AEB_PARTIAL */ { 0.40f, 5, { { W_TRI, 0.00f, 65, 1400, 0 }, { W_TRI, 0.10f, 65, 1800, 0 }, { W_TRI, 0.20f, 65, 1400, 0 },
                                    { W_TRI, 0.30f, 65, 1800, 0 }, { W_TRI, 0.40f, 65, 1400, 0 } } },
    // Full braking: the pulses Damien picked, 4 dB quieter ("a bit too loud, the tone is right")
    /* AEB_FULL    */ { 0.38f, 7, { { W_SQ, 0.00f, 60, 1400, 0 }, { W_SQ, 0.09f, 60, 1800, 0 }, { W_SQ, 0.18f, 60, 1400, 0 },
                                    { W_SQ, 0.27f, 60, 1800, 0 }, { W_SQ, 0.36f, 60, 1400, 0 }, { W_SQ, 0.45f, 60, 1800, 0 },
                                    { W_SQ, 0.54f, 60, 1400, 0 } } },
};

static const int RATE = 22050;
static const int MAXSAMPLES = RATE;  // every cue fits in 1 s

struct Built { unsigned char wav[44 + MAXSAMPLES * 2]; unsigned bytes; float vol; };
static Built g_built[CUE_COUNT];

static void put32(unsigned char *p, unsigned v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put16(unsigned char *p, unsigned v) { p[0] = v; p[1] = v >> 8; }

static float sample(const Note &n, int k, int len)
{
    float t = (float)k / RATE, d = n.ms / 1000.0f, w = 6.2831853f * n.f0 * t;
    switch (n.w) {
    case W_BELL: {  // a struck bell: a few partials that die away at different speeds
        float v = sinf(w) + 0.35f * sinf(2 * w) * expf(-t * 14 / d) + 0.12f * sinf(2.76f * w) * expf(-t * 20 / d);
        return v * fminf(1.0f, t / 0.004f) * expf(-t * 7 / d) * 0.6f;
    }
    case W_SOFT:  // round sine, slow in and out
        return sinf(w) * fminf(1.0f, k / (RATE * 0.015f)) * fminf(1.0f, (len - k) / (RATE * 0.03f));
    case W_GLIDE: {  // sine that slides from f0 to f1 (phase integrated, no clicks)
        float f = n.f0 + (n.f1 - n.f0) * k / len;
        float ph = 6.2831853f * (n.f0 * t + (n.f1 - n.f0) * 0.5f * t * t / d);
        (void)f;
        return sinf(ph) * fminf(1.0f, k / (RATE * 0.01f)) * fminf(1.0f, (len - k) / (RATE * 0.03f));
    }
    case W_TRI: {
        float p = fmodf(k * n.f0 / RATE, 1.0f);
        return (4 * fabsf(p - 0.5f) - 1) * fminf(1.0f, k / (RATE * 0.003f)) * fminf(1.0f, (len - k) / (RATE * 0.006f));
    }
    case W_PIP:  // flat sine with short soft edges
        return sinf(w) * fminf(1.0f, k / (RATE * 0.008f)) * fminf(1.0f, (len - k) / (RATE * 0.012f));
    case W_SQ: {
        float p = fmodf(k * n.f0 / RATE, 1.0f);
        return (p < 0.5f ? 0.6f : -0.6f) * fminf(1.0f, k / (RATE * 0.002f)) * fminf(1.0f, (len - k) / (RATE * 0.004f));
    }
    }
    return 0;
}

static void build(int cue)
{
    const CueDef &c = CUES[cue];
    static float mix[MAXSAMPLES];
    memset(mix, 0, sizeof mix);
    int used = 0;
    for (int i = 0; i < c.n; i++) {
        const Note &n = c.notes[i];
        int o = (int)(n.at_s * RATE), len = RATE * n.ms / 1000;
        for (int k = 0; k < len && o + k < MAXSAMPLES; k++) mix[o + k] += sample(n, k, len);
        if (o + len > used) used = o + len;
    }
    used += RATE / 10;  // a short tail of silence so the end never cuts the decay
    if (used > MAXSAMPLES) used = MAXSAMPLES;
    float gain = c.vol * (g_volume / 0.35f);
    Built &b = g_built[cue];
    short *pcm = (short *)(b.wav + 44);
    for (int k = 0; k < used; k++) {
        float v = mix[k] * gain;
        pcm[k] = (short)(fmaxf(-1.0f, fminf(1.0f, v)) * 32000);
    }
    b.bytes = (unsigned)used * 2;
    unsigned char *g = b.wav;
    memcpy(g, "RIFF", 4); put32(g + 4, 36 + b.bytes); memcpy(g + 8, "WAVEfmt ", 8);
    put32(g + 16, 16); put16(g + 20, 1); put16(g + 22, 1); put32(g + 24, RATE);
    put32(g + 28, RATE * 2); put16(g + 32, 2); put16(g + 34, 16);
    memcpy(g + 36, "data", 4); put32(g + 40, b.bytes);
    b.vol = g_volume;
}

void play_cue(int cue)
{
    if (g_volume <= 0 || cue < 0 || cue >= CUE_COUNT) return;
    Built &b = g_built[cue];
    if (b.bytes == 0 || b.vol != g_volume) build(cue);
    // each cue owns its buffer: async PlaySound reads it while it plays
    PlaySoundA((LPCSTR)b.wav, NULL, SND_MEMORY | SND_ASYNC | SND_NODEFAULT);
}
