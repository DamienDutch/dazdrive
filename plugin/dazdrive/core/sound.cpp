// Short feedback tones (mode, ACC distance, lane assist on/off), played
// asynchronously from a WAV built in memory (winmm PlaySound; works under
// Proton). One tone sequence at a time; a new one replaces the old.
#include "suite.h"
#include <math.h>
#include <mmsystem.h>

float g_volume = 0.35f;

static unsigned char g_wav[44 + 2 * 22050 * 2];  // up to 2 s, 22.05 kHz mono 16-bit

static void put32(unsigned char *p, unsigned v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put16(unsigned char *p, unsigned v) { p[0] = v; p[1] = v >> 8; }

// freqs: n tones of ms each with a short gap; 0 Hz = silence.
void play_tones(const float *freqs, int n, int ms)
{
    if (g_volume <= 0 || n <= 0) return;
    const int rate = 22050, gap = rate * 40 / 1000, len = rate * ms / 1000;
    int total = n * (len + gap);
    if (total > 2 * rate) total = 2 * rate;
    short *pcm = (short *)(g_wav + 44);
    int i = 0;
    for (int t = 0; t < n && i < total; t++) {
        for (int k = 0; k < len && i < total; k++, i++) {
            float env = fminf(1.0f, fminf(k, len - k) / (rate * 0.008f));  // 8 ms fade in/out, no clicks
            pcm[i] = freqs[t] > 0 ? (short)(sinf(6.2831853f * freqs[t] * k / rate) * env * g_volume * 32000) : 0;
        }
        for (int k = 0; k < gap && i < total; k++, i++) pcm[i] = 0;
    }
    unsigned bytes = (unsigned)i * 2;
    memcpy(g_wav, "RIFF", 4); put32(g_wav + 4, 36 + bytes); memcpy(g_wav + 8, "WAVEfmt ", 8);
    put32(g_wav + 16, 16); put16(g_wav + 20, 1); put16(g_wav + 22, 1); put32(g_wav + 24, rate);
    put32(g_wav + 28, rate * 2); put16(g_wav + 32, 2); put16(g_wav + 34, 16);
    memcpy(g_wav + 36, "data", 4); put32(g_wav + 40, bytes);
    PlaySoundA((LPCSTR)g_wav, NULL, SND_MEMORY | SND_ASYNC | SND_NODEFAULT);
}
