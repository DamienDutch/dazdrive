# Low-speed behavior of real truck AMTs (research notes)

Confidence key: [V] verified in a page/PDF that was opened; [S] seen only in a search snippet; [P] patent (generic/ZF/Volvo, not proof of production tuning); [I] inference.
Big caveat: no manufacturer driver manual publicly spells out clutch-slip curves, creep speeds in km/h or the stopping downshift logic. Numbers below are what could be found.

## Sources opened
- Detroit DT12 transmission card (Daimler, 2020): https://assets.wellertruck.com/reference-materials/Driver Information/Detroit DT12 Transmission Card.pdf [V]
- Eaton UltraShift PLUS quick reference TRDR0941 (2014): https://roadtrains.com.au/wp-content/uploads/2019/09/Eaton-Ultrashift-Drivers-Instruction-Manual.pdf [V]
- Eaton "Urge to Move"/"Blended Pedal" press: https://vehicleservicepros.com/12117425 [V] (also https://www.fleetequipmentmag.com/eaton-low-speed-enhancements-automated-transmissions/ [S])
- Volvo I-Shift (US) operator's manual: https://hhrvresource.com/sites/default/files/dfiles/VolvoOperatorsManualI_Shift_PV77620906026.pdf [V]
- Volvo I-Shift drive modes fact sheet (2022): https://stpi.it.volvo.com/STPIFiles/Volvo/FactSheet/DRM-E,DRM-BE_Eng_05_331482000.pdf [V]
- Volvo I-Shift crawler gears: https://www.truckinginfo.com/156889/driving-volvos-i-shift-with-crawler-gears [V]; https://www.verkehrsrundschau.de/nachrichten/nfz-fuhrpark/zwei-neue-kriechgaenge-bei-volvo-2992990 [S]
- Scania Opticruise Spezialtest (Eurotransport): https://www.eurotransport.de/fahrzeuge/lkw/getriebetest-scania-r-opticruise-im-spezialtest/ [V]
- Scania Opticruise brochure FR (04/2017): https://www.scania.com/content/dam/scanianoe/market/fr/products-and-services/trucks/opticruise.pdf [V]
- Scania G33CM article: https://www.roadtrains.com.au/articles/the-evolution-of-scania-opticruise-g33-cm/ [V]
- ZF TraXon via Liebherr "press the accelerator and protect your clutch": https://www.liebherr.com/en-ro/mobile-and-crawler-cranes/customer-magazine/my-tip/press-the-accelerator-and-protect-your-clutch-4322895 [V]
- ZF TraXon page: https://www.zf.com/products/en/special_vehicles/product_highlights/product_highlight_traxon/product_highlight_traxon.html [V, thin]
- Patent US6626796 (ZF maneuvering mode): https://patents.google.com/patent/US6626796 [P,V]
- Patent EP1303423A1 (low speed maneuvering control, "urge torque"): https://patents.google.com/patent/EP1303423A1/cg-RX-API-DMAC7.html [P,V]
- Patent (Coventry/Eaton-type enhanced creep, 0-75% pedal): https://pureportal.coventry.ac.uk/en/publications/selectable-enhanced-creep-control-mode-for-automated-clutch-and-v/ [P]
- Volvo patent US8136647 clutch control (search snippet only, page 403): https://patents.justia.com/patent/8136647 [S,P]
- Commercial Motor 2005 Renault AMT "how it works": https://archive.commercialmotor.com/article/17th-november-2005/50/how-it-works [S only, page empty on fetch]
- Iveco S-Way/X-Way/T-Way brochure lines: https://viewer.ipaper.io/iveco-hq/ame-en/iveco-s-way/?Page=21 [S only]
- MAN TipMatic 2 (DM/RM program): https://www.truck.man.eu/za/en/man-world/technology-and-competence/technology/man-tipmatic-2/MAN-TipMatic-2.html [S only, 403]
- Mercedes PowerShift 3: https://www.mercedes-benz-trucks.com/en_IE/models/actros/driving-dynamics/propulsion-technology.html [S: "crawl function", maneuvering]; crane operator quote via https://www.transportengineer.org.uk/content/features/rapid-response-gets-a-lift [S]

## 1. Creep (brake released, no throttle)
Two fundamentally different philosophies exist:

A. Idle-governor creep, clutch CLOSED (no slip): Volvo "Engine Idle Governor driving mode" (US manual) [V]: engine at idle, "automatic clutch remains closed and there is no slipping or overheating. The idle governor will adjust the torque to maintain idle speed and constant vehicle speed". Driver picks gear for speed (gear 1 = slowest; tapping brake downshifts one gear; manual buttons too); accelerator raises speed; "clutch is only disengaged when the vehicle is stopped or the brake pedal is fully applied". Volvo "Launch Control" (EU fact sheet) [V]: "possible to drive the vehicle forward with the idle regulator... saves unnecessary downshifts", EBS brakes auto-engage on direction change, Auto Hold active. Speed = idle rpm x ratio: crawler gears reach 0.6 mph (~1 km/h) [V], std 12-speed 1st about 2 mph (3 km/h) [V]. Renault Optidriver Xtended crawler: 1.55 km/h at 1000 rpm [S]. Scania normal mode: clutch stays closed so idle speed is possible; only opens if resistance threatens to stall the engine [V Eurotransport].

B. Eaton "Urge to Move" / Detroit "Creep Mode" (US), clutch partially engaged at idle: Urge to Move: in gear + service brake released, truck starts moving without throttle, then "creeps at a constant speed while engine is at idle", like a passenger car [V]. Eaton older UltraShift: creep only in MANUAL/LOW (default disabled in Drive): tap throttle until you feel the clutch engage, release, truck idles along in current gear; exit by throttle or service brake [V]. Detroit DT12: shift N->D/R, release brakes, "briefly depress and release the accelerator" and creep mode engages; display warns before creep mode is aborted for high clutch temperature; extended creep triggers "clutch abuse" alerts [V]. Creep limits are thermal: Detroit disables it by clutch temp; Eaton "CA" warning, then opens clutch / removes throttle control to cool [V]. Patent EP1303423 [P]: with brake fully released, controlled slip gives creep about 5-10 km/h (gear/weight dependent); clutch torque is inversely related to brake effort, down to fully open at standstill; mode only in R/1st/2nd(3 lowest) and below ~16 km/h; "urge torque" > idle torque so the engine is commanded above idle. Hence the car-DCT-style creep is torque-based, not speed-based. Patent US6626796 (ZF) [P]: creep speed is set by the controller (fixed maneuver speeds picked by hold time/tap count), not by pedal; brake OR gas pedal cancels it.
Hills: Detroit Hill Start Aid holds service brakes on grades >3% for 3 s after full stop [V]; Eaton HSA holds up to 3 s on >1% grade, in forward uphill/reverse downhill [V]. Detroit tip: never hold on a slope with throttle ("slipping the clutch") [V]. Volvo manual: never hold uphill with accelerator, clutch overheats [V]. Scania two-pedal: Hill Hold standard, engages briefly when brake pressed at standstill [V, brochure].

## 2. Maneuvering mode
- Scania Opticruise: activated by pushing the control lever forward ~1 s (small "m" in display), pull back to cancel; auto-cancels when gearbox is in the high range group [V]. Clutch is finer-metered than usual; "there is neither an rpm limit nor does the full pedal travel apply" [V]. Key difference to normal mode: in maneuvering mode the clutch OPENS at idle / when the driver lifts off throttle (so no creep, no pushing against brake), in normal mode it stays closed at idle [V]. Full gear speed range stays available. Two-pedal version has a clutch overload warning for too much driving below idle speed [V]. Tester: sensitivity comparable to ZF or Mercedes [V]. Clutch on Demand variant has an optional clutch pedal [S].
- ZF TraXon (DAF etc.): maneuvering mode (tortoise icon) forces 1st (in some applications only R1) and lets the clutch slip for very slow speed, "extremely sensitive to accelerator movements" [V ZF]; Liebherr crane data: with clutch fully closed min speed in 1st/off-road gear 1.2 km/h, in maneuvering mode 0.6 km/h [V]; clutch lamp stays lit in maneuvering mode as reminder not to stay long; "CL" symbol = overheated [V]. Without the mode "clutch is designed to be fairly in or out ... on-off switch" [S trade pub]. Clutch closes more as throttle increases ("coupling moment" rises with pedal); depends on gradient, weight, gear and mode [V].
- MAN TipMatic 2: DM/RM maneuvering program, "partly with clutch slip", limited to first two gears, short-term use only [S].
- Renault Optidriver (2005): maneuvering function desensitizes accelerator and allows more clutch slip than normal; clutch temp monitored, dash warning [S].
- Iveco Hi-Tronix: "maneuvering (creep) mode... working like a torque-converter transmission", plus rocking function and off-road mode [S].
- Mercedes PowerShift 3: "crawl function" for comfortable moving off, fast forward/reverse changeover; maneuvering mode cuts rev range to about 1000 rpm for finer throttle control (one operator) [S]. Older Actros had a Rangiermodus switch [S].
- Eaton: "Blended Pedal" lets accelerator position directly control clutch engagement at idle (variable speed with engine at idle) [V]. Patent: enhanced creep effective over 0-75 % pedal [P].
- Volvo: "Change Direction" (up to 30 km/h; accelerator brakes the vehicle and reverses without brake pedal), "Terrain Brake" (holds brake when accelerator released, below 4 km/h), both off-road packages [V]. Paver Assist: roll very slowly, press + [S].

## 3. Stopping (downshifts while braking, clutch opening)
Hard public data is scarce. Facts:
- Scania: when braking from low speed (roundabout, lights), neutral is engaged instead of a low range gear; the system prepares the best gear for pull-away, engaged as soon as throttle is pressed [S Scania text via oemoffhighway, repeated in search; not opened]. Downshift points at low speed are flexible ("1200 or only 1000 rpm") [V]. Downshifts under combined service+engine brake at low speed felt "ruppig" (rough) in the test [V].
- Volvo (patent snippet) [S,P]: at idle the vehicle keeps creeping in low gear; clutch disengages only when brake is touched; example threshold 750 rpm with 600 rpm idle; disengagement rate depends on brake force (hard stop = faster release). Volvo manual: tap brake to downshift one gear; clutch opens only when vehicle stopped or brake fully applied [V].
- EP1303423 [P]: clutch torque declines with brake effort, fully open at standstill.
- Detroit/Eaton launch gear after stop is re-selected from mass and grade (not always 1st) [V]; Eaton LOW mode used for coming to a stop (max engine braking, downshifts at earliest opportunity) [V].
- No source stated explicit skip-shift-and-declutch logic for stopping; Eaton says "system may skip shift in Drive" generally [V]. Inference [I]: common practice is to decouple (clutch open/neutral) rather than step through every gear.

## 4. Starting
- Start gear chosen from load (mass estimate), grade, axle ratio; driver can lower it; system overrides inappropriate picks (Eaton, Detroit, Volvo) [V]. "Not uncommon to start in a gear other than first" (Detroit) [V]. Too-high start gear = clutch overheating warnings [V Volvo]. Scania 40 t on 13 % grade: picks big crawler as start gear but tester noted it downshifts 1-2 gears too many on first 13-15 % climbs [V].
- Clutch take-up depends on throttle: more pedal = more clutch torque; brisk start reduces wear [V Liebherr/ZF]. Pressing both pedals gives abrupt launch/rock risk (Eaton warns) [V].
- Hill hold: see section 1; Detroit 3 s, Eaton up to 3 s, Scania brief hold on brake at standstill.

## 5. Other features worth copying
- Rocking / Rock Free (Eaton, Iveco, Volvo off-road "faster clutch release, easier rocking") [V/S]; Scania auto-rocking when drive wheel speed differs from front [S].
- Auto neutral after parking brake (Detroit 10 s, Eaton) [V]; clutch abuse protection tiers (Detroit/Eaton/Scania/ZF) [V].
- Off-road mode (Volvo, ZF "off-road gear"): lower start gear, quicker clutch release, multi-downshifts [V].
- Kickdown at low speed: Eaton/Detroit performance mode returns to economy when throttle reduced [V].

## 6. What is rated best / trade-offs
- Scania two-pedal Opticruise praised for smooth, jerk-free start and shifts, best-in-class on extreme hill, maneuvering mode as sensitive as ZF/Mercedes [V Eurotransport]. Opticruise earlier criticized for low-speed maneuvering being the AMT weak point until two-pedal maneuvering mode (2009) [V roadtrains].
- Trade-offs: closed-clutch idle governor (Volvo) = zero wear, no heat, but speed is quantized by gear (min ~1-3 km/h) and engine drives against brake; slipping clutch creep (Eaton/Detroit) = smooth like torque converter, but heat limit and time cap; maneuvering mode with pedal-controlled slip (Scania/ZF/MAN) = most precise, but clutch wear and short-use warnings; crawler gears (Volvo/Renault) = best of both mechanically, extra hardware.
- True torque-converter autos remain the smoothest creep; AMTs imitate by clutch control [S].

## Recommended for a sim (summary)
1. Stop/low speed: use a 'clutch open' regime below ~(idle rpm + margin): once rpm falls to ~700-750 (idle 600) or speed < ~3-4 km/h with brake applied, open the clutch (rate proportional to brake force); do not stay closed in gear 1. Do not downshift sequentially to 1st: hold the current gear and decouple, then pick launch gear from load/grade at standstill.
2. Creep: torque-based slip (urge torque slightly above idle torque, ~ walking pace 3-6 km/h, only gears 1-2/R, brake reduces torque, throttle takes over) plus a time/heat budget; fall back to idle-governor closed clutch for crawl gears.
3. Maneuvering mode: pedal -> clutch-torque map over 0-75 % pedal travel, engine held near idle, clutch opens at idle/lift-off, auto-exit above ~16 km/h or high range.
4. Starting: launch gear by mass/grade; hill hold 3 s; clutch torque follows throttle.
