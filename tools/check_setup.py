#!/usr/bin/env python3
"""Headless check of dazdrive-setup.html: builds the page, runs its ?selftest in headless Chromium and compares the
page's ini for the recommended era (~2010 = the shipped preset, so no overrides) with `gen_ini.py standard`.
Usage: tools/check_setup.py [chromium binary]. Exit 1 on any failure."""
import json, os, re, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
import gen_ini
chromium = sys.argv[1] if len(sys.argv) > 1 else shutil.which("chromium-browser") or shutil.which("chromium") or shutil.which("google-chrome")
if not chromium:
    raise SystemExit("no chromium found")
tmp = tempfile.mkdtemp(prefix="dazdrive-setup-")
page = os.path.join(tmp, "dazdrive-setup.html")
subprocess.run([sys.executable, os.path.join(HERE, "gen_ini.py"), "html", "-o", page], check=True)
standard = subprocess.run([sys.executable, os.path.join(HERE, "gen_ini.py"), "standard"], check=True, capture_output=True, text=True).stdout
dom = subprocess.run([chromium, "--headless=new", "--disable-gpu", "--no-sandbox", f"--user-data-dir={tmp}/profile",
                      "--virtual-time-budget=15000", "--dump-dom", f"file://{page}?selftest"], capture_output=True, text=True, timeout=60).stdout
i = dom.rfind('<pre id="selftest">')  # the last one: the script source mentions the id too
if i < 0:
    raise SystemExit("selftest output not found in the DOM (page error?)")
raw = dom[i + len('<pre id="selftest">'):dom.find("</pre>", i)]
out = json.loads(raw.replace("&quot;", '"').replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&"))
fails = list(out["fails"])
# the page's ini for the recommended era must be gen_ini standard plus that era's overrides (comments differ by design)
def values(text):
    return [l for l in text.splitlines() if l and not l.startswith("#")]
rec = [e for e in gen_ini.ERAS if e.get("recommended")][0]
expect, sec = [], None
for l in values(standard):
    if l.startswith("["):
        sec = l[1:-1]
    elif "=" in l:
        k = l.split("=", 1)[0]
        if k in rec["over"].get(sec, {}):
            l = f"{k}={gen_ini.fmt(rec['over'][sec][k])}"
    expect.append(l)
if out["rec"] != rec["id"]:
    fails.append(f"page recommends {out['rec']}, gen_ini {rec['id']}")
if values(out["iniRec"]) != expect:
    fails.append(f"ini {rec['id']} != gen_ini standard + era overrides")
    for x, y in zip(values(out["iniRec"]), expect):
        if x != y:
            print(f"  page: {x}\n  gen : {y}")
            break
shutil.rmtree(tmp, ignore_errors=True)
if fails:
    print("FAIL:", ", ".join(fails))
    sys.exit(1)
print(f"setup page ok: selftest passed, era {rec['id']} == standard preset + overrides")
