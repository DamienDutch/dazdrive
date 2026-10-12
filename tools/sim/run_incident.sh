#!/bin/bash
# Build the incident recorder test with MinGW (podman, like plugin/dazdrive/build.sh) and run it under wine. ~30 s.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); root=$(realpath "$here/../..")
work=$(mktemp -d)
podman run --rm -v "$root:/w:Z" -v "$work:/out:Z" -w /w/plugin/dazdrive registry.fedoraproject.org/fedora:44 bash -c \
  "dnf -y -q install mingw64-gcc-c++ >/dev/null 2>&1 && x86_64-w64-mingw32-g++ -O1 -std=c++17 -w -I../sdk/include -I. \
   -o /out/sim_incident.exe ../../tools/sim/sim_incident.cpp core/log.cpp core/incident.cpp -static -static-libgcc -static-libstdc++"
cd "$work" && WINEDEBUG=-all wine sim_incident.exe
