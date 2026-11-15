// One key=value line of an ini text, replaced or added, the rest byte for byte the same (comments, blank lines, line endings, BOM).
// Used by config_live_set (core/config.cpp) to keep the player's ini in step with a setting changed from the phone, and by the host test.
#pragma once
#include <stdlib.h>
#include <string.h>

// Returns a malloc'd copy of text with [sec] key = val: the last "key=" line of any [sec] block gets its value replaced (its leading
// text up to the "=" and the line ending stay); with no such line the key is added after the last non-blank line of the last [sec]
// block; with no [sec] at all a new block is appended. *outn = its length (not NUL terminated by contract, but a NUL follows). NULL = no memory.
// ini_with2 also says in *had whether the key had a line already (a player's ini may spell a setting in the new or in the legacy way).
static char *ini_with2(const char *text, size_t n, const char *sec, const char *key, const char *val, size_t *outn, int *had)
{
    const char *eol = n && memchr(text, '\n', n) ? (const char *)memchr(text, '\n', n) : NULL;
    const char *nl = eol && eol > text && eol[-1] == '\r' ? "\r\n" : "\n";
    size_t sl = strlen(sec), kl = strlen(key), vl = strlen(val), nll = strlen(nl);
    size_t hit_a = 0, hit_b = 0, hit_eq = 0;   // the last matching line: [a,b) with the value starting at hit_eq
    int hit = 0, in = 0, have_sec = 0;
    size_t ins = 0;                            // after the last non-blank line of the last [sec] block
    for (size_t a = 0; a < n;) {
        size_t b = a;
        while (b < n && text[b] != '\n') b++;
        size_t e = b < n ? b + 1 : b;          // [a,e) with its line ending, [a,b) without the \n
        size_t t0 = a, t1 = b;
        if (t0 == 0 && t1 >= 3 && !memcmp(text, "\xEF\xBB\xBF", 3)) t0 = 3;
        while (t0 < t1 && (text[t0] == ' ' || text[t0] == '\t')) t0++;
        while (t1 > t0 && (text[t1 - 1] == ' ' || text[t1 - 1] == '\t' || text[t1 - 1] == '\r')) t1--;
        if (t0 < t1 && text[t0] == '[') {
            const char *close = (const char *)memchr(text + t0, ']', t1 - t0);
            in = close && (size_t)(close - (text + t0) - 1) == sl && !memcmp(text + t0 + 1, sec, sl);
            if (in) { have_sec = 1; ins = e; }
        } else if (in && t0 < t1) {
            ins = e;
            if (text[t0] != '#' && text[t0] != ';') {
                const char *eq = (const char *)memchr(text + t0, '=', t1 - t0);
                if (eq) {
                    size_t k0 = t0, k1 = (size_t)(eq - text);
                    while (k1 > k0 && (text[k1 - 1] == ' ' || text[k1 - 1] == '\t')) k1--;
                    if (k1 - k0 == kl && !memcmp(text + k0, key, kl)) {
                        hit = 1; hit_a = a; hit_b = b; hit_eq = (size_t)(eq - text) + 1;
                    }
                }
            }
        }
        a = e;
    }
    if (had) *had = hit;   // 1 = the key had a line in a [sec] block (its value is replaced), 0 = it is added
    char *o = (char *)malloc(n + sl + kl + vl + 16);
    if (!o) return NULL;
    size_t m = 0;
    if (hit) {   // keep what is before the value (and the spaces after "="), drop the old value, keep a "\r" and the "\n"
        size_t v0 = hit_eq;
        while (v0 < hit_b && (text[v0] == ' ' || text[v0] == '\t')) v0++;
        memcpy(o, text, v0); m = v0;
        memcpy(o + m, val, vl); m += vl;
        size_t tail = hit_b > hit_a && text[hit_b - 1] == '\r' ? hit_b - 1 : hit_b;
        memcpy(o + m, text + tail, n - tail); m += n - tail;
    } else if (have_sec) {
        bool eol_ok = ins > 0 && text[ins - 1] == '\n';   // the last line of the block had no line ending (end of file)
        memcpy(o, text, ins); m = ins;
        if (!eol_ok) { memcpy(o + m, nl, nll); m += nll; }
        memcpy(o + m, key, kl); m += kl; o[m++] = '=';
        memcpy(o + m, val, vl); m += vl;
        memcpy(o + m, nl, nll); m += nll;
        memcpy(o + m, text + ins, n - ins); m += n - ins;
    } else {
        memcpy(o, text, n); m = n;
        if (m && o[m - 1] != '\n') { memcpy(o + m, nl, nll); m += nll; }
        if (m) { memcpy(o + m, nl, nll); m += nll; }
        o[m++] = '['; memcpy(o + m, sec, sl); m += sl; o[m++] = ']'; memcpy(o + m, nl, nll); m += nll;
        memcpy(o + m, key, kl); m += kl; o[m++] = '=';
        memcpy(o + m, val, vl); m += vl;
        memcpy(o + m, nl, nll); m += nll;
    }
    o[m] = 0;
    *outn = m;
    return o;
}
static char *ini_with(const char *text, size_t n, const char *sec, const char *key, const char *val, size_t *outn)
{
    return ini_with2(text, n, sec, key, val, outn, NULL);
}
