---
name: Bug report
about: Something behaves oddly or not at all
labels: bug
---

**What happened, and what did you expect?**


**Setup**
- DazDrive version (first lines of `plugins\dazdrive\dazdrive.log`):
- ETS2 version (Steam beta temporary_1_60?):
- Truck, engine and gearbox (mods?):
- Trailer: yes / no. Navigation route active: yes / no.
- Wheel / controller and which buttons you pressed:

**When**
In-game time or odometer when it happened, and roughly how often it happens.

**Files**
`dazdrive.log` is overwritten every time the game starts, so copy `plugins\dazdrive\dazdrive.log`
(and `dazdrive.log.1` if it exists) to somewhere safe **before you start the game again**, then attach
the copy.

- About emergency braking (AEB)? Also attach the folder `plugins\dazdrive\aeb\` (zipped).
- About driving behavior? Pick "Bug hunt" under Logging on the setup page (csv traces only exist at
  log_level 2), drive again and attach the zipped `.csv` files from the same folder (`trueshift.csv`,
  `cruise.csv`, `lane.csv`, `creep.csv`).
