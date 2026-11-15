// The ini's names. The code's param tables keep one internal (module, key) per setting; dazdrive.ini calls it [section] key
// (lowercase snake_case words, sub-sections as [module.subsection], see tools/ini_names.tsv). This header holds the map
// (ini_names.inc, generated from the tsv), the reading of one value (numbers, true/false, the words of a choice) and the scan
// of a whole ini text. Used by config.cpp and by the host test (tools/sim/test_ini_names.cpp), which is why it has no game code.
//
// A player's old ini keeps loading: a key under a plain [module] header ([cruise] acc_short_s) is the legacy spelling of the same
// setting, found by its internal name. A (section, key) in the new layout is found first.
#pragma once
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct ini_name {
    const char *mod, *old_key;   // the internal name: the module's name and the key of its param table (also the legacy ini spelling)
    const char *sec, *key;       // the public name
    char audience;               // u user, a advanced, i internal (not written into a generated ini, still read)
    char type;                   // n number, s switch (true/false), c choice (words)
    const char *choices;         // "value=word value=word", choice rows only
};
static const ini_name k_ini_names[] = {
#include "ini_names.inc"
};
enum { INI_N = (int)(sizeof k_ini_names / sizeof k_ini_names[0]) };

static inline int ini_ieq(const char *a, const char *b)
{
    for (;; a++, b++) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        if (!*a) return 1;
    }
}
// the public (section, key), case-insensitive
static inline const ini_name *ini_by_new(const char *sec, const char *key)
{
    for (int i = 0; i < INI_N; i++)
        if (ini_ieq(k_ini_names[i].sec, sec) && ini_ieq(k_ini_names[i].key, key)) return &k_ini_names[i];
    return NULL;
}
// the internal (module, key) = the legacy spelling
static inline const ini_name *ini_by_old(const char *mod, const char *key)
{
    for (int i = 0; i < INI_N; i++)
        if (!strcmp(k_ini_names[i].mod, mod) && !strcmp(k_ini_names[i].old_key, key)) return &k_ini_names[i];
    return NULL;
}
// the internal module name when name is one (ini_ieq), else NULL
static inline const char *ini_module(const char *name)
{
    for (int i = 0; i < INI_N; i++)
        if (ini_ieq(k_ini_names[i].mod, name)) return k_ini_names[i].mod;
    return NULL;
}
// is this a section name of the new layout
static inline int ini_has_section(const char *name)
{
    for (int i = 0; i < INI_N; i++)
        if (ini_ieq(k_ini_names[i].sec, name)) return 1;
    return 0;
}

// The words true/false/on/off/yes/no (any case) are 1 and 0 for every key. A choice row also takes its own words.
static inline int ini_word_value(const ini_name *row, const char *w, double *d)
{
    if (row && row->type == 'c') {
        const char *p = row->choices;
        while (*p) {
            const char *eq = strchr(p, '=');
            if (!eq) break;
            const char *e = eq + 1;
            while (*e && *e != ' ') e++;
            size_t n = (size_t)(e - (eq + 1));
            if (strlen(w) == n) {
                bool same = true;
                for (size_t i = 0; i < n; i++) if (tolower((unsigned char)w[i]) != eq[1 + i]) same = false;
                if (same) { *d = atof(p); return 1; }
            }
            p = *e ? e + 1 : e;
        }
    }
    static const struct { const char *w; double v; } k_bool[] = { { "true", 1 }, { "false", 0 }, { "on", 1 }, { "off", 0 }, { "yes", 1 }, { "no", 0 } };
    for (size_t i = 0; i < sizeof k_bool / sizeof k_bool[0]; i++)
        if (ini_ieq(w, k_bool[i].w)) { *d = k_bool[i].v; return 1; }
    return 0;
}
// A value as text (already trimmed, may be edited in place: a decimal comma becomes a dot). Returns 0 = not a value,
// 1 = value, 2 = value followed by text that is ignored. A word of a choice or true/false/on/off/yes/no needs no number.
static inline int ini_value(const ini_name *row, char *val, double *d)
{
    if (ini_word_value(row, val, d)) return 1;
    for (char *c = val; *c; c++) if (*c == ',') *c = '.';   // a decimal comma (1,5) means 1.5 here
    char *end;
    *d = strtod(val, &end);
    if (end == val) return 0;
    while (*end == ' ' || *end == '\t') end++;
    return *end ? 2 : 1;
}
// how to write a value in a generated ini: true/false for a switch at 0 or 1, the word for a choice that has one, else the number
static inline void ini_format(const ini_name *row, double v, char *out, size_t n)
{
    if (row && row->type == 's' && (v == 0 || v == 1)) { snprintf(out, n, v ? "true" : "false"); return; }
    if (row && row->type == 'c') {
        for (const char *p = row->choices; *p;) {
            const char *eq = strchr(p, '=');
            if (!eq) break;
            const char *e = eq + 1;
            while (*e && *e != ' ') e++;
            if (atof(p) == v) { snprintf(out, n, "%.*s", (int)(e - (eq + 1)), eq + 1); return; }
            p = *e ? e + 1 : e;
        }
    }
    snprintf(out, n, "%.6g", v);
}

static inline char *ini_trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) *--e = 0;
    return s;
}

// Reads an ini text and hands every setting to h:
//   h.warn(const char *msg)                          a problem with the file itself (long line, unknown section)
//   h.extra_section(const char *name) -> int         a section that something else reads (the local extras): skipped silently
//   h.unknown_key(const char *sec, const char *key)  a key that is not a setting
//   h.set(const char *sec, const char *key, const ini_name *row, int rc, double d, const char *val)
//       a setting: sec and key as the file spells them, row = what it is, rc/d from ini_value, val the text
template <class H>
static void ini_scan(FILE *f, H &h)
{
    char line[512], sec[96] = "";
    int in_sec = 0, first = 1;
    const char *legacy = NULL;   // the internal module name when the section header is a plain module name
    while (fgets(line, sizeof line, f)) {
        char *s = line;
        if (first && !memcmp(s, "\xEF\xBB\xBF", 3)) s += 3;   // UTF-8 byte order mark (Notepad): else "[core]" is not seen
        first = 0;
        size_t len = strlen(line);
        if (len == sizeof line - 1 && line[len - 1] != '\n') {   // longer than the buffer: use the start, drop the rest
            char w[96];
            snprintf(w, sizeof w, "ini: a line longer than %d characters was cut", (int)sizeof line - 2);
            h.warn(w);
            int c;
            while ((c = fgetc(f)) != EOF && c != '\n') { }
        }
        s = ini_trim(s);
        if (!*s || *s == '#' || *s == ';') continue;
        if (*s == '[') {
            char *e = strchr(s, ']');
            if (e) *e = 0;
            snprintf(sec, sizeof sec, "%s", ini_trim(s + 1));
            legacy = ini_module(sec);
            in_sec = legacy || ini_has_section(sec);
            if (!in_sec && !h.extra_section(sec)) {
                char w[160];
                snprintf(w, sizeof w, "ini: unknown section [%s]", sec);
                h.warn(w);
            }
            continue;
        }
        char *eq = strchr(s, '=');
        if (!eq || !in_sec) continue;
        *eq = 0;
        char *key = ini_trim(s), *val = ini_trim(eq + 1);
        const ini_name *row = ini_by_new(sec, key);
        if (!row && legacy) row = ini_by_old(legacy, key);
        if (!row) { h.unknown_key(sec, key); continue; }
        double d = 0;
        int rc = ini_value(row, val, &d);
        h.set(sec, key, row, rc, d, val);
    }
}
