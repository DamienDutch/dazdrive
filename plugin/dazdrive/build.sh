#!/bin/bash
# Build the suite DLL with MinGW g++ in a throwaway Fedora container (no host packages needed).
# Usage: plugin/dazdrive/build.sh [install]   install = copy the DLL into ETS2's plugins folder.
set -euo pipefail
here=$(realpath "$(dirname "$0")")
root=$(realpath "$here/..")
python3 "$root/../tools/gen_ini.py" inc -o "$here/core/preset_standard.inc"   # the player preset as a C table (core/config.cpp)
name=$(sed -n 's/^#define SUITE_NAME "\([^"]*\)".*/\1/p' "$here/core/suite.h")
# Local extras (core/dev.cpp + dev.h, not part of a release): only when they are there and DAZDRIVE_PUBLIC is not set
# (tools/make_release.sh sets it). Without them the DLL has no trace of them (DEV_* in core/suite.h are empty).
srcs=$(cd "$here" && ls core/*.cpp modules/*.cpp | grep -v '^core/dev\.cpp$' | tr '\n' ' ')
# The flavor lands in the DLL as the string "dazdrive-flavor=<flavor>" (core/main.cpp); make_release.sh accepts "public" only.
flavor=dev
if [ "${DAZDRIVE_PUBLIC:-0}" = 1 ]; then flavor=public; fi
extra="-DDAZDRIVE_FLAVOR=$flavor"
if [ -f "$here/core/dev.cpp" ] && [ -f "$here/core/dev.h" ] && [ "${DAZDRIVE_PUBLIC:-0}" != 1 ]; then
  srcs="$srcs core/dev.cpp"; extra="$extra -DDAZDRIVE_DEV"; echo "build: with the local extras (core/dev.cpp)"
fi
echo "build: flavor $flavor"
podman run --rm -v "$root:/w:Z" -w /w/dazdrive registry.fedoraproject.org/fedora:44 bash -c \
  "dnf -y -q install mingw64-gcc-c++ >/dev/null 2>&1 && x86_64-w64-mingw32-g++ -O2 -Wall -Wextra \
   -Wno-unused-parameter -Wno-missing-field-initializers -std=c++17 $extra -I../sdk/include -I. -shared \
   -o $name.dll $srcs suite.def -static -static-libgcc -static-libstdc++ -lwinmm -ldinput8 -ldxguid"
ls -l "$here/$name.dll"
if [ "${1:-}" = install ]; then
  dst="$HOME/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/win_x64/plugins"
  cp "$here/$name.dll" "$dst/"
  echo "installed in $dst"
fi
