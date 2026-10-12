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

**When**
In-game time or odometer when it happened, and roughly how often it happens.

**Files**
When something goes wrong, DazDrive saves that moment (20 s before, 15 s after) in
`plugins\dazdrive\incidents\`. Zip that folder and attach it, together with `plugins\dazdrive\dazdrive.log`
(the previous session's log is `dazdrive.prev.log`).

- Nothing in the incidents folder, but it felt wrong? Say so. For driving behavior you can also pick
  "Bug hunt" under Logging on the setup page (csv traces only exist at log_level 2), drive again and
  attach the zipped `.csv` files from the same folder.
