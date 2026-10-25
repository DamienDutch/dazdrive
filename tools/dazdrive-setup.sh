#!/bin/bash
# DazDrive setup: walks through the important choices per module and writes dazdrive.ini.
# Linux / Proton users: run it in a terminal. Windows users: just open dazdrive.ini in Notepad, every setting has
# a comment line above it (see README.txt).
#
#   dazdrive-setup.sh [--base FILE] [--out FILE] [--check]
#     --base FILE   start from this ini (default: dazdrive.ini next to this script, or tools/../release preset;
#                   pass dazdrive-dev.ini for the developer's settings, or your installed ini to change it)
#     --out FILE    where to write (default: the game's plugins/dazdrive folder if it exists, else ./dazdrive.ini)
#     --check       only verify that every question below still exists in the base ini (used by make_release.sh)
#
# Uses whiptail or dialog when present, plain prompts otherwise. Nothing is written until the last step; an
# existing output file is copied to FILE.bak first.
set -u
here=$(cd "$(dirname "$0")" && pwd)
base=""; out=""; check=0
while [ $# -gt 0 ]; do
  case "$1" in
    --base) base=$2; shift 2 ;;
    --out) out=$2; shift 2 ;;
    --check) check=1; shift ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
if [ -z "$base" ]; then
  for c in "$here/dazdrive.ini" "$here/../dazdrive.ini" "$here/../release/dazdrive.ini"; do [ -f "$c" ] && base=$c && break; done
fi
[ -f "$base" ] || { echo "no base ini found (use --base FILE)" >&2; exit 1; }

# ---- the questions: section|key|kind|label    kind: b = on/off, n = number, c:a/b/c = choice
Q='core|log_level|c:0/1/2|How much goes into dazdrive.log (1 is fine; 2 also writes big csv files)
trueshift|enabled|b|TrueShift: the gearbox logic
trueshift|dash_letter|b|Show the gearbox mode (D, E, S, M) on the dashboard
trueshift|eco_roll|b|Eco-roll: neutral on gentle descents (Eco and Normal modes)
trueshift|smooth_stop|b|Smooth stop: open the clutch around idle when braking to a stop
trueshift|lookahead|b|Use the navigation route ahead (climbs, tops, junctions)
trueshift|hint_hold_s|n|Seconds a gear chosen with the shift-hint buttons is held
trueshift|tones|b|Short tone when the gearbox mode changes
cruise|enabled|b|Cruise module (smooth throttle, ACC extras)
cruise|smooth|b|Smooth cruise throttle (no bursts after shifts)
cruise|speed_limit|b|Follow speed limits with the cruise set speed
cruise|speed_limit_offset_kmh|n|Set speed = limit plus this (km/h)
cruise|auto_hold|b|Auto Hold: stay braked after stopping until you give gas
cruise|stop_go|b|ACC keeps going in a jam (stop and go)
cruise|own_acc|b|Own, smoother ACC speed target
cruise|acc_cycle_up|b|ACC distance key steps close, middle, far (the game: far, middle, close)
cruise|curves|b|Slow down gently before sharp bends
cruise|junctions|b|Slow down before junction turns and roundabouts
cruise|descent|b|Brake gently ahead of descents to hold the set speed
cruise|predictive|b|Carry a little speed into climbs, ease off before tops
cruise|emergency_hazards|b|Hazard lights by themselves after an emergency stop
cruise|step_tap_kmh|n|A tap on cruise + / - changes the set speed by this (0 = the game'"'"'s step)
cruise|min_kmh|n|Lowest speed at which cruise can be switched on (the game: 30)
lane|mode|c:0/1/2|Lane keeping: 0 = the game'"'"'s own, 1 = calm, 2 = model-based (experimental)
lane|tone|b|Tone when lane keeping switches on/off
creep|enabled|b|Crawl / maneuver mode (slipping clutch in 1, 2 and R)
creep|start_on|b|Crawl switched on when the game starts
creep|creep_kmh|n|Crawl speed (km/h)'

# ---- read the ini: tab separated section, key, value, help (the comment line above the key)
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
awk '/^\[/ {s=$0; gsub(/[\[\]]/,"",s); next}
     /^#/ {h=$0; sub(/^# ?/,"",h); next}
     /^[A-Za-z0-9_]+=/ {i=index($0,"="); printf "%s\t%s\t%s\t%s\n", s, substr($0,1,i-1), substr($0,i+1), h; h=""}' "$base" > "$tmp/ini.tsv"

get() { awk -F'\t' -v s="$1" -v k="$2" -v f="$3" '$1==s && $2==k {print $f; exit}' "$tmp/ini.tsv"; }

missing=0
while IFS='|' read -r sec key kind label; do
  [ -z "$sec" ] && continue
  if [ -z "$(get "$sec" "$key" 2)" ]; then echo "question [$sec] $key is not in $base" >&2; missing=1; fi
done <<< "$Q"
[ $check = 1 ] && { [ $missing = 0 ] && echo "setup questions match $(basename "$base")"; exit $missing; }

# ---- UI
ui=plain
if command -v whiptail >/dev/null && [ -t 0 ]; then ui=whiptail; elif command -v dialog >/dev/null && [ -t 0 ]; then ui=dialog; fi
dlg() { if [ $ui = whiptail ]; then whiptail "$@" 3>&1 1>&2 2>&3; else dialog --stdout "$@"; fi; }

short() { echo "$1" | cut -c1-110; }

answers="$tmp/answers.tsv"; : > "$answers"
set_answer() { printf '%s\t%s\t%s\n' "$1" "$2" "$3" >> "$answers"; }

ask_bool() {  # sec key label help current
  local ans
  if [ $ui = plain ]; then
    printf '\n%s\n  (%s)\n' "$3" "$4"
    read -r -p "  on? [$( [ "$5" = 0 ] && echo y/N || echo Y/n )] " ans
    case "$ans" in [yYjJ]*) ans=1 ;; [nN]*) ans=0 ;; *) ans=$5 ;; esac
    set_answer "$1" "$2" "$ans"
  else
    if dlg $( [ "$5" = 0 ] && echo --defaultno ) --title "$1" --yesno "$3\n\n$4\n\nOn?" 18 76; then set_answer "$1" "$2" 1; else set_answer "$1" "$2" 0; fi
  fi
}
ask_num() {  # sec key label help current
  local ans
  if [ $ui = plain ]; then
    printf '\n%s\n  (%s)\n' "$3" "$4"
    read -r -p "  value [$5] " ans
    [ -z "$ans" ] && ans=$5
  else
    ans=$(dlg --title "$1" --inputbox "$3\n\n$4" 18 76 "$5") || ans=$5
  fi
  if ! [[ "$ans" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then echo "not a number, keeping $5" >&2; ans=$5; fi
  set_answer "$1" "$2" "$ans"
}
ask_choice() {  # sec key label help current options(a/b/c)
  local ans opts=${6//\// }
  if [ $ui = plain ]; then
    printf '\n%s\n  (%s)\n' "$3" "$4"
    read -r -p "  one of $opts [$5] " ans
    [ -z "$ans" ] && ans=$5
  else
    local items=() o
    for o in $opts; do items+=("$o" ""); done
    ans=$(dlg --title "$1" --default-item "$5" --menu "$3\n\n$4" 22 76 6 "${items[@]}") || ans=$5
  fi
  case " $opts " in *" $ans "*) ;; *) echo "not one of $opts, keeping $5" >&2; ans=$5 ;; esac
  set_answer "$1" "$2" "$ans"
}

echo "DazDrive setup. Base: $base"
[ $ui = plain ] && echo "(no whiptail/dialog found: plain questions; Enter keeps the value in brackets)"
if [ $ui != plain ]; then dlg --title "DazDrive setup" --msgbox "Walks through the important settings. Everything else stays as in:\n$base\n\nNothing is written until the end." 12 70; fi

# modules: skip the detail questions of a module that was switched off in this run
off_sections=" "
while IFS='|' read -r -u 3 sec key kind label; do
  [ -z "$sec" ] && continue
  case "$off_sections" in *" $sec "*) [ "$key" != enabled ] && continue ;; esac
  cur=$(get "$sec" "$key" 3); help=$(get "$sec" "$key" 4)
  case "$kind" in
    b) ask_bool "$sec" "$key" "$label" "$help" "$cur" ;;
    n) ask_num "$sec" "$key" "$label" "$help" "$cur" ;;
    c:*) ask_choice "$sec" "$key" "$label" "$help" "$cur" "${kind#c:}" ;;
  esac
  if [ "$key" = enabled ] && [ "$(awk -F'\t' -v s="$sec" '$1==s && $2=="enabled" {v=$3} END{print v}' "$answers")" = 0 ]; then off_sections="$off_sections$sec "; fi
done 3<<< "$Q"

# ---- write: the base with the answers filled in (comments stay)
if [ -z "$out" ]; then
  steam="$HOME/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/win_x64/plugins/dazdrive"
  [ -d "$steam" ] && out="$steam/dazdrive.ini" || out="$PWD/dazdrive.ini"
fi
result="$tmp/result.ini"
awk -F'\t' -v af="$answers" 'FILENAME==af {a[$1 "\t" $2]=$3; next}
  { line=$0 }
  /^\[/ { s=$0; gsub(/[\[\]]/,"",s) }
  /^[A-Za-z0-9_]+=/ { i=index($0,"="); k=substr($0,1,i-1); if ((s "\t" k) in a) line=k "=" a[s "\t" k] }
  { print line }' FS='\t' "$answers" FS='\n' "$base" > "$result"
# the first awk file is tab-separated, the second a plain line reader; FS switching above does that

if [ $ui = plain ]; then
  printf '\nWrite %s ? [Y/n] ' "$out"; read -r yn
  case "$yn" in [nN]*) echo "nothing written"; exit 0 ;; esac
else
  dlg --title "DazDrive setup" --yesno "Write the settings to:\n$out" 10 76 || { echo "nothing written"; exit 0; }
fi
mkdir -p "$(dirname "$out")" 2>/dev/null
[ -f "$out" ] && cp "$out" "$out.bak" && echo "old file kept as $out.bak"
cp "$result" "$out" && echo "written: $out  (restart the game for the changes to take effect)"
