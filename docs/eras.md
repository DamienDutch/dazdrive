# Truck eras on the setup page: when the real thing appeared

Source for the year presets in `tools/gen_ini.py` (ERAS). European heavy trucks, first series availability at any
of the big brands; "~" because the brands trail each other by a few years. Checked online; the rest from
general knowledge, marked (k).

The setup page offers six "truck years" as presets: **~1995 Classic, ~2000 Radar cruise, ~2010 Modern, ~2015 Looks ahead,
~2020 Assisted, 2024 Current**. Each year switches on the features that existed in a series truck by then. Feature names below are the ones
used on the setup page.

Stages: **stable** = tested a lot, **beta** = built and tested a little, **experimental** = not proven yet. A stage moves up as a feature
proves itself. A year shows the weakest stage of the features it switches on.

| Feature (setup-page name) | Stage | On from | First in a series truck | Note |
|---|---|---|---|---|
| TrueShift (automated manual gearbox) | stable | ~1995 | 1995 Scania Opticruise; ZF AS-Tronic ~1997; Volvo I-Shift 2002; Mercedes PowerShift 2006 | [Roadtrains](https://www.roadtrains.com.au/articles/the-evolution-of-scania-opticruise-g33-cm/), [Fleet Owner](https://www.fleetowner.com/refrigerated-transporter/article/21226491/i-shift-debut), [Commercial Motor 2013](https://archive.commercialmotor.com/article/26th-september-2013/16/mercedes-actros) |
| SteadyCruise (calm cruise control) | stable | ~1995 | 1990s (k): Mercedes SK/Actros 1996 Telligent | |
| Smooth stop (no judder when braking to a halt) | stable | ~2000 | comes with the automated gearbox (1995) | |
| Downhill speed control (Bremsomat: engine brake/retarder hold the set speed) | stable | ~2000 | 1990s (k): Mercedes SK/Actros 1996 Telligent | |
| Hill start assist | stable | ~2000 | ~2000-2005 (k): Actros Telligent hill holder; standard on every brand by ~2010 | |
| Eco-roll (neutral on gentle descents) | stable | ~2010 | 2006 Volvo I-Roll (k), 2008 Mercedes EcoRoll with PowerShift as standard (k) | Mercedes page gives no year: [MB](https://mercedes-benz-trucks.com/en_SG/models/actros/driving-dynamics/propulsion-technology.html) |
| Emergency stop signal (hazards flash on hard braking) | stable | ~2010 | ~2010 (k): Actros 2011 generation; UN R48 made it mandatory with the 07 series (adopted 2019/2020) | [GAR](https://globalautoregs.com/documents/21509) |
| AEB (emergency braking) | stable | ~2010 | 2006 Mercedes Active Brake Assist (k); EU mandatory on new trucks November 2015 (k) | DazDrive's own AEB replaces the game's |
| Gearbox looks ahead (predictive shifting) | stable | ~2015 | 2012 Scania Active Prediction, 2012 Mercedes PPC, 2012/2013 Volvo I-See; DAF ~2015 (k) | [Commercial Motor 2012](https://archive.commercialmotor.com/article/23rd-august-2012/35/longer-sighted-lorries), [eurotransport](https://www.eurotransport.de/fahrzeuge/lkw/vorausschauender-tempomat-i-see-von-volvo-soll-fuenf-prozent-sprit-sparen/) |
| Cruise looks ahead (predictive cruise, GPS + map) | stable | ~2015 | same as above | same sources |
| ACC (radar distance cruise) | beta | ~2015 (DazDrive's own ACC) | 2000 Mercedes Actros ART (k); common by ~2005 | the game's own ACC works in every year |
| Stop & Go (ACC keeps going in a jam) | beta | ~2015 | 2011 Mercedes Actros Proximity Control Assist with stop-and-go (k); other brands ~2016 | stop-and-go confirmed, year not: [MB](https://mercedes-benz-trucks.com/en_HK/models/new-actros/efficiency/greater-safety.html) |
| Auto Hold (hold at standstill) | beta | ~2015 | same as Stop & Go | |
| Slow for bends (bends, roundabouts, junctions in the predictive cruise) | stable | ~2020 | 2019 Mercedes Actros PPC on country roads | [RoadStars](https://roadstars.mercedes-benz-trucks.com/en_GB/magazine/transport/01-2019/predictive-powertrain-control-the-intelligent-cruise-control.html) |
| JunctionSense (slows for junctions and roundabouts) | experimental | ~2020 | same as above | |
| Crawl mode (slipping clutch, no pedal) | experimental | ~2020 | 2009 two-pedal Opticruise with maneuvering mode | [Roadtrains](https://www.roadtrains.com.au/articles/the-evolution-of-scania-opticruise-g33-cm/) |
| Lane centering (Level 2: ACC + steering) | experimental | ~2020 | 2019 Mercedes Actros Active Drive Assist, first series truck | [Fleet News](https://www.fleetnews.co.uk/news/truck-news/2018/09/06/new-mercedes-actros-will-launch-with-semi-autonomous-active-drive-assist), [NZ Trucking](https://nztrucking.co.nz/?p=114225) |
| Follow speed limits (ISA) | beta | 2024 | EU GSR: mandatory on new types July 2022, all new trucks July 2024 (k); on the options list from ~2019 with PPC maps | |
| Lane departure warning | (the game's own) | always | EU mandatory on new trucks November 2015 (k) | the game has it already |

## What this means for the presets

Realistic cut-offs: ~1995 (automated gearbox + plain cruise), ~2000 (downhill speed control, hill start, smooth stop), ~2010 (eco-roll, emergency stop
signal, AEB), ~2015 (predictive gearbox and cruise, own ACC with Stop & Go, Auto Hold), ~2020 (bends/roundabouts/junctions, lane centering, crawl mode:
Scania had it in 2009, but it is experimental here and would drag ~2015 down to experimental), 2024 (ISA).

The presets follow these years, experimental parts included where they belong. The setup page preselects ~2010 (Modern), the newest year whose
features are all stable. It moves up when the beta features of ~2015 have proven themselves.

**The ini shipped in the zip equals the ~2010 (Modern) year.** On: TrueShift, SteadyCruise, Smooth stop, Downhill speed control, Hill start assist, Eco-roll,
Emergency stop signal and AEB. Off, and coming on in later years: Gearbox looks ahead and Cruise looks ahead (from ~2015), ACC, Stop & Go and Auto Hold
(from ~2015), Slow for bends (from ~2020), Crawl mode, JunctionSense and Lane centering (from ~2020, experimental), Follow speed limits / ISA (2024).
Opening the shipped ini on the setup page therefore shows the ~2010 year with no changes.
