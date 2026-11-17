# Leadership brief: Spot confined-space entry demo (8 minutes)

One page for the presenter. Setup, shells and fallbacks are in
[RUNBOOK.md](RUNBOOK.md) (presenter cheat sheet); this is what to say.
Screens: the Isaac Sim window (DCV, presentation mode) and the HUD side by
side. Shell A types the four prompts; shell B stages the gas spike. Timings
are from the leadership rehearsal on 2026-09-28 (5 reps + 1 clean run, GUI
mode); the robot time for the whole story is about 2.5 minutes.

## Before the audience (shell B)

`bash reset_spot.sh --reset-ventilation`. Check: HUD says NOT STARTED, Spot
stands at spawn facing the tank, battery green. Keep shell A at the
`operator>` prompt.

## The flow

| Clock | Beat | Do | Say |
|---|---|---|---|
| 0:00 | Why | nothing | "Tank cleaning means sending a person through a 1.3 m door into a space that can kill them in minutes. Today a trained entrant does it under a permit. We want no one inside: the robot takes the readings and the risk." |
| 0:45 | Permit | A: "Prepare permit CSE-0427: the attendant is at the entry, isolation and lockout-tagout are in place, rescue plan: on-site standby team. Then run the pre-entry atmospheric test at the tank." Then type `y` when the console asks you to confirm the attestations (~33 s) | "The rules do not change because a robot does the work. The permit needs isolation, lockout-tagout, an attendant and a rescue plan before the robot may approach." |
| 1:30 | First test | (same turn) | "Spot stops outside the door and reaches in with a gas probe, half a meter and one meter in, at three heights, oxygen first, then flammables, then toxics. Hydrogen sulfide is about 52 ppm at floor level, five times the limit. Entry denied." |
| 2:15 | Ventilate, early re-test | A: "Start ventilation, wait 30 seconds, then re-test. Don't wait for the purge." (~45 s) | "We ventilate, as a crew would. I am asking for the re-test early on purpose." When the amber PURGE HOLD appears: "Every reading at the door is now within permissible entry limits, but 11 percent of the contamination is still in the tank. The system holds the entry until the purge target, 8 percent, is met. The screen and the assistant quote the same number." |
| 3:15 | Authorized | A: "Wait for the purge target, then re-test." (~12 s) | "Target reached, H2S about 4 ppm. Only now is entry authorized." |
| 3:40 | Entry | A: "Proceed with the confined space entry." Then type `y` when the console asks to confirm the entry. | "The robot does not decide to go in: a person confirms it at the console. Spot goes in on fixed waypoints around the pipes and reads the air the whole way. The view cuts inside the tank." |
| 3:45 | Excursion | B: `bash inject_gas.sh` when the HUD map shows Spot at waypoint 1 | "I am staging a hydrogen sulfide release now; the screen marks it as staged. Watch who reacts: not the language model. A deterministic safety guard stops the robot within milliseconds and backs it out." (alarm about 1.2 s after Enter, retreat about 12 s) |
| 4:15 | Explanation | let the assistant finish | "The permit is aborted. Re-entry needs a new permit and a new test. Every reading, decision and the staged spike are in the audit log." |
| 4:45 | What is real | nothing | See the caveats below. Say them plainly. |
| 6:00 | The ask / Q&A | | Two minutes of questions; the robot sits outside, the HUD keeps the incident on screen. |

If the assistant already authorized entry at the end of the ventilation turn
(it waited and re-tested by itself, 3 of 20 sessions), skip "Wait for the
purge target". For a calm ending instead of the excursion, skip shell B and
say "Walk back out to the entry point." after the entry (entry ~38 s, exit
~43 s, permit COMPLETED).

## Honest caveats (say these)

- **This is a simulation.** NVIDIA Isaac Sim on AWS: the robot, the tank, the
  physics and the camera views are simulated.
- **The gas is simulated too.** The readings, the probe and the blower are a
  model, and the spike is staged by me. None of these numbers are field
  measurements.
- **Spot is not ATEX or IECEx certified.** Base Spot is not rated for
  explosive atmospheres; a real entry into a tank with flammable vapour needs
  certified equipment or an inerted, gas-free tank, and a site hazard review.
- **The same agent code is meant to drive the real robot.** The assistant and
  its safety guard talk to the simulator through the same ROS 2 interface
  that the open source spot_ros2 driver exposes for a real Spot. That has
  not been run against spot_ros2 or hardware yet; that is the next step when
  the robot arrives.
- **The language model proposes; code enforces.** The permit gate, the purge
  gate and the safety guard are deterministic code. The model explains and
  sequences, and cannot override them.

## Never say

That the tank or the entry "is safe" (say "within permissible entry limits");
that Spot or its payload is intrinsically safe or certified; that any named
operator uses this; that the readings are real.
