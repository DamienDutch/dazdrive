#!/bin/bash
# Publish the setup page of a release to GitHub Pages (branch gh-pages): setup/<version>/ built from the release's tag,
# setup/latest/ = the same page, and the version list in index.html. The page is generated from that tag's param tables
# (tools/gen_ini.py html), so it always matches the DLL of that release. Run after the tag exists; pushes gh-pages.
# Usage: tools/publish_setup_page.sh 0.6.0-beta.5 [--no-push]
set -euo pipefail
ver=${1:?usage: publish_setup_page.sh <version> [--no-push]}
push=1; [ "${2:-}" = --no-push ] && push=0
repo=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
git -C "$repo" rev-parse -q --verify "refs/tags/$ver" >/dev/null || { echo "no tag $ver" >&2; exit 1; }
tmp=$(mktemp -d); trap 'git -C "$repo" worktree remove --force "$tmp/src" 2>/dev/null; git -C "$repo" worktree remove --force "$tmp/pages" 2>/dev/null; rm -rf "$tmp"' EXIT
git -C "$repo" worktree add -q --detach "$tmp/src" "$ver"
git -C "$repo" fetch -q origin gh-pages
git -C "$repo" worktree add -q "$tmp/pages" origin/gh-pages
python3 "$tmp/src/tools/gen_ini.py" html -o "$tmp/page.html"
mkdir -p "$tmp/pages/setup/$ver" "$tmp/pages/setup/latest"
cp "$tmp/page.html" "$tmp/pages/setup/$ver/index.html"
cp "$tmp/page.html" "$tmp/pages/setup/latest/index.html"
# the version list: newest first (sort -V), latest = the one just published
cd "$tmp/pages"
items=$(ls -d setup/*/ | sed 's#setup/##; s#/##' | grep -v '^latest$' | sort -rV | sed 's#.*#<li><a href="setup/&/">&</a></li>#')
python3 - "$ver" "$items" <<'EOF'
import re, sys
ver, items = sys.argv[1], sys.argv[2]
s = open("index.html").read()
latest = '<li><a href="setup/latest/">Latest release</a> (%s)</li>\n' % ver
s = re.sub(r"<ul>.*?</ul>", "<ul>\n" + latest + items + "\n</ul>", s, flags=re.S)
open("index.html", "w").write(s)
EOF
git add -A
git diff --cached --quiet && { echo "unchanged: setup/$ver/ is already published"; exit 0; }
git -c user.name=DamienDutch -c user.email=109442798+DamienDutch@users.noreply.github.com commit -q -m "Setup page for $ver (built from the tag)"
if [ $push = 1 ]; then git push -q origin HEAD:gh-pages; echo "published: https://damiendutch.github.io/dazdrive/setup/$ver/"; else echo "committed locally (not pushed)"; fi
