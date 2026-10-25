#!/bin/bash
# Assemble a DazDrive release folder + zip. Prepares only: nothing is published, nothing is copied into the game.
#
#   tools/make_release.sh <version> [--no-build] [--skip-sims] [--exe PATH]
#
# builds the public DLL (plugin/dazdrive/build.sh with DAZDRIVE_PUBLIC=1: podman + MinGW), runs the simulators, generates
# the ini preset and the browser setup page from the param tables (tools/gen_ini.py: they cannot drift), runs the page's
# self-test, checks the player texts for leftovers, and writes release/dazdrive-<version>/ and
# release/dazdrive-<version>.zip, laid out like the game's plugins folder:
#   README.txt  LICENSE  BUILD-INFO.txt
#   plugins/dazdrive.dll
#   plugins/dazdrive/dazdrive.ini (standard preset)  plugins/dazdrive/dazdrive-setup.html
# <version> must equal SUITE_VERSION in core/suite.h (the DLL carries it in the log).
# --no-build reuses plugin/dazdrive/dazdrive.dll; it is accepted only if that DLL says "dazdrive-flavor=public".
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(realpath "$here/..")
ver=${1:-}; shift || true
build=1; sims=1; exe=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-build) build=0 ;;
    --skip-sims) sims=0 ;;
    --exe) exe=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$ver" ] || { sed -n 2,14p "$0"; exit 2; }
src_ver=$(sed -n 's/^#define SUITE_VERSION "\([^"]*\)".*/\1/p' "$root/plugin/dazdrive/core/suite.h")
[ "$ver" = "$src_ver" ] || { echo "version $ver is not SUITE_VERSION ($src_ver) in core/suite.h: change one of them" >&2; exit 1; }

dll="$root/plugin/dazdrive/dazdrive.dll"
if [ $build = 1 ]; then DAZDRIVE_PUBLIC=1 "$root/plugin/dazdrive/build.sh"; fi   # never the local extras (core/dev.*)
[ -f "$dll" ] || { echo "no DLL at $dll" >&2; exit 1; }
# Read the strings once into a variable: a pipe into grep -q would stop strings early (SIGPIPE) and, under pipefail,
# count as "no hit" (the check silently passed a dev DLL once).
dll_strings=$(strings -a "$dll")
grep -qxF -- "$ver" <<<"$dll_strings" || { echo "the DLL does not carry the version string $ver: stale build? (rebuild without --no-build)" >&2; exit 1; }
grep -qxF -- "dazdrive-flavor=public" <<<"$dll_strings" || { echo "the DLL is not a public build (no dazdrive-flavor=public): rebuild without --no-build" >&2; exit 1; }
if grep -qF -- "dazdrive-flavor=dev" <<<"$dll_strings"; then echo "the DLL is a dev build: rebuild without --no-build" >&2; exit 1; fi

if [ $sims = 1 ]; then
  echo "== simulators"
  for s in run.sh run_lane.sh run_creep.sh run_cruise.sh; do
    if "$root/tools/sim/$s" > /dev/null 2>&1; then echo "  $s green"; else echo "  $s FAILED (run it by hand; --skip-sims to ignore)" >&2; exit 1; fi
  done
fi

out="$root/release/dazdrive-$ver"
rm -rf "$out"; mkdir -p "$out"
pd="$out/plugins/dazdrive"
mkdir -p "$pd"
cp "$dll" "$out/plugins/dazdrive.dll"
python3 "$here/gen_ini.py" standard -o "$pd/dazdrive.ini"
python3 "$here/gen_ini.py" html -o "$pd/dazdrive-setup.html"
# a full release (no -beta/-rc suffix) ships quiet: the betas ship with bug hunt on (log_level 2, csv traces) so reports have data
case "$ver" in *-*) ;; *) grep -q '^log_level=0' "$pd/dazdrive.ini" || { echo "full release $ver but the standard ini has bug hunt on (log_level): switch it off in tools/gen_ini.py" >&2; exit 1; } ;; esac
"$here/dazdrive-setup.sh" --base "$pd/dazdrive.ini" --check   # the terminal setup stays in the repo, not in the zip
echo "== setup page self-test"
if command -v chromium-browser >/dev/null 2>&1 || command -v chromium >/dev/null 2>&1 || command -v google-chrome >/dev/null 2>&1; then
  python3 "$here/check_setup.py" || { echo "check_setup.py FAILED" >&2; exit 1; }
else
  echo "  WARNING: no chromium found, the setup page self-test was skipped" >&2
fi
sed "s/@VERSION@/$ver/g" "$root/release/README-player.txt" > "$out/README.txt"
cp "$root/LICENSE" "$out/LICENSE"
grep -q "Deppelover" "$out/LICENSE" || { echo "LICENSE lacks the upstream (Deppelover) notice" >&2; exit 1; }
grep -q "SCS Software" "$out/LICENSE" || { echo "LICENSE lacks the SCS SDK notice" >&2; exit 1; }

# Leftovers a player should never see: developer names, work-log dates, build names, private links, local extras.
echo "== leftover check"
# Work-log dates look like "24-10" (day-month, the project runs from September): any dd-09..dd-12 is one. "1-2 gear" is not.
datepat='\b[0-3]?[0-9]-(09|1[0-2])\b'
if grep -n -i -E "damien|$datepat|dev\.[0-9]+|pre\.[0-9]+|claude|home assistant|ha_out|rides\.jsonl|live\.json|/home/|tools/sim|notes/|docs/|\bts[0-9]+[a-z]\b" \
     "$out/README.txt" "$pd/dazdrive.ini" "$pd/dazdrive-setup.html" | grep -v -i 'github.com/DamienDutch' | grep -v 'Copyright'; then
  echo "leftovers in the player files (above): fix the source texts first" >&2; exit 1
fi
if grep -i -E "Damien [0-9]|$datepat|rides\.jsonl|ha_out|tools/sim|notes/|docs/" <<<"$dll_strings" | grep -v -i 'github.com/DamienDutch' | head -3; then
  echo "leftovers in the DLL strings (above): fix the help strings first" >&2; exit 1
fi
echo "  clean"

# the game build the DLL was checked against (offline pattern check on the exe)
[ -n "$exe" ] || exe="$HOME/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/win_x64/eurotrucks2.exe"
suite_check=""
if [ -f "$exe" ]; then
  suite_check=$(python3 "$here/check_suite.py" "$exe" 2>&1) || { echo "check_suite.py: no layout matches $exe:" >&2; echo "$suite_check" >&2; exit 1; }
fi
{
  echo "DazDrive $ver"
  echo "built:   $(date -u +%Y-%m-%dT%H:%MZ)"
  echo "sha256:  $(sha256sum "$out/plugins/dazdrive.dll" | cut -d' ' -f1)"
  echo "game:    Euro Truck Simulator 2 1.60 (Steam beta temporary_1_60) only"
  echo "ini: plugins/dazdrive/dazdrive.ini (standard preset), generated from the source tables"
  if [ -n "$suite_check" ]; then
    echo "check_suite.py on $(basename "$exe") ($(stat -c %s "$exe") bytes):"
    sed 's/^/  /' <<<"$suite_check"
  else
    echo "check_suite.py: no game exe found (pass --exe PATH)"
  fi
} > "$out/BUILD-INFO.txt"

(cd "$root/release" && rm -f "dazdrive-$ver.zip" && zip -qr "dazdrive-$ver.zip" "dazdrive-$ver")
echo "== done: $out and $out.zip"
ls -l "$out"
echo "Nothing is published: upload the zip as a GitHub Release when it has been driven."
