# DazDrive in Convoy and TruckersMP

Tags: [src] = read in the linked source (via a WebFetch summary, not raw text), [inferred] = reasoning.

## Outcome
- **TruckersMP: do not use DazDrive there.** TruckersMP does not allow tools that change gameplay (their rule 2.1), and its client scans game files and memory.
  DazDrive patches game code in memory (gearbox, cruise, brakes), so it falls under that rule and is detectable. The README says so: singleplayer and Convoy only.
- **SCS Convoy: allowed in principle, not tested yet.** Convoy is hosted by the players themselves, and each client simulates only its own truck, so DazDrive affects
  nobody else. Still to check: that the hooks behave in a host-synced session (cruise and AEB act on the player's own truck only).

## Convoy (SCS built-in multiplayer)
- Convoy mod support (1.42, 2021-10-21) talks only about MODs (content packages): max 70 mods per session, vehicles/trailers/cargo
  need traffic models, sound mods are local unless configured, missing mods come from the Workshop.
  Nothing about plugins/SDK, no cheating or driving-aid statement. [src] https://blog.scssoft.com/2021/10/ets2ats-142-release.html
- SCS telemetry SDK wiki page has no word on multiplayer, Convoy or restrictions. [src] https://modding.scssoft.com/wiki/Documentation/Engine/SDK/Telemetry
- Third-party description: the server exports one description of map/DLC/mods and clients must match it. A plugin DLL is not a mod
  in that list, so there is no match check for it. [src, secondary] https://www.gameserverkings.com/knowledge-base/american-truck-simulator/ats-mods-convoy-dedicated-server/
- Not found: any SCS statement on the plugin dialog in Convoy, or on cheating/assistive tools in Convoy. Patch notes 1.50-1.61 not checked line by line.
- Technical: each client simulates its own truck; Convoy syncs state to others. Our hooks change only our own gearbox/cruise/brakes
  (inputs to our own physics), no other player's truck or the host's world. [inferred] Whether the game blocks plugin loading in a
  session is unverified; the plugin loads at game start, before any session. [inferred]
- Convoy hosts can force things (speed limiter is a server setting, per the GSK guide) so a host may not want driving aids. [src, secondary]

## TruckersMP
- Independent mod hooking into the game, not affiliated with SCS. [src, secondary] https://www.survivalservers.com/wiki/Euro_Truck_Simulator_2_Convoy_vs_TruckersMP
- ETS2LA on the TruckersMP forum, thread "Is the ETS2LA (Lane Assist) mod banned in TruckersMP?" (https://forum.truckersmp.com/index.php?%2Ftopic%2F118593-is-the-ets2la-lane-assist-mod-banned-in-truckersmp%2F=):
  only the search-result summary was readable. A user reply (not confirmed staff) says the software is not prohibited per se; if it glitches and causes collisions you are
  punished as usual; own risk. [src, secondary, weak]
- Konvoi VTC tracker says it reads the same SCS telemetry plugin under TruckersMP, so telemetry-reading plugins do load there. [src, secondary] https://docs.konvoi.gg/reference/supported-games
- Key difference from pure telemetry: DazDrive patches game code in memory (gearbox, cruise, brakes). TruckersMP has anti-tamper/anti-cheat style rules and its client
  scans memory (see the rules below). An AEB/ACC that brakes is also visible to others as braking behavior; a bug that causes collisions is punishable. [inferred]

## TruckersMP rules (read: rules as updated 06-10-2026, ToS, Privacy Policy)
No word on SDK plugins, DLLs, telemetry or driving aids: nothing explicitly allowed or forbidden. Risk points:
- Rules 2.1 (6 months - permanent): "Using any kind of tool to change gameplay, including but not limited to ...
  trainers or cheat engine ... or anything similar"; punishments "on a case basis without justification".
- Rules 1.11: "Any ambiguity falls on the side of TruckersMP"; 1.1: player responsible for "any other setting
  altering game physics"; 2.2/2.5: a collision or unsafe driving caused by an ACC/AEB/lane assist is the player's fault.
- Privacy 2.2.1: the client scans "game-related ... files and memory" and "running processes, drivers and other
  executable code" to detect "tools to work around rules ... or game physics"; 2.2.19 logs "Plugin usage".
- ToS/EULA reverse-engineering clauses cover the TMP client and its network, not the SCS game.
- Nothing about SCS Convoy (not their rules).
Verdict: TrueShift alone most defensible, cruise/AEB and lane assist riskier, whole DLL (memory patches) closest to
"tool to change gameplay" and detectable. DazDrive is therefore singleplayer (or Convoy) only. Questions that only TMP staff could settle: SDK plugins
writing own controls allowed? memory-patching driving aids? are unknown DLLs in plugins/ flagged? exemption (5.2)?

## Precedents
- car_automatic_transmission (deppelover, our upstream, MIT): README Multiplayer section says "Single player only. Do not use with TruckersMP or Convoy."
  No reason given. [src, via fetch summary] https://github.com/deppelover/car_automatic_transmission
- ETS2LA docs front page: nothing about TruckersMP/Convoy. [src] https://docs.ets2la.com/ ; its crash page only says to use a game version TruckersMP supports. [src, secondary via search]
- MonoCruise: no statement found.
