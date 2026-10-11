#!/bin/bash
# Build a plugin DLL with MinGW in a throwaway Fedora container (no host packages needed).
# Usage: plugin/build.sh <dir with .c and .def>   -> writes <name>.dll next to the source.
set -euo pipefail
dir=$(realpath "${1:-$(dirname "$0")/upstream}")
c=$(ls "$dir"/*.c | head -1); name=$(basename "$c" .c)
podman run --rm -v "$dir:/src:Z" -w /src registry.fedoraproject.org/fedora:44 bash -c \
  "dnf -y -q install mingw64-gcc >/dev/null && x86_64-w64-mingw32-gcc -O2 -Wall -Wextra -shared \
   -o $name.dll $name.c $name.def -static -lkernel32 -lm"
ls -l "$dir/$name.dll"
