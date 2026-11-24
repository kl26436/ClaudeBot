# Robot A code: handoff

Updated 2026-11-23, after the rev3 CAD. Read this before the plan. It records where the build stands, the
decisions that override the plan's text, and what is still open.

## Where things are

| Thing | Location |
|---|---|
| Code | this repository, branch `robot-a`. Nothing has been pushed anywhere. |
| What the CAD asks of the code | `docs/cad/ROBOT_A_SOFTWARE_HANDOFF.md` (CAN map and joint list beside it). Newer than the design where they differ. |
| Design | `docs/superpowers/specs/2026-11-21-robot-a-code-design.md`, exported from the Claude Doc "Robot A code design" (rev 36) |
| Plan | `docs/superpowers/plans/2026-11-21-robot-a-code.md` (19 tasks, all done) |
| Wiring | `docs/wiring.md`, from the rev3 CAN map |
| Unconfirmed values | `docs/open-numbers.md`: every `PLACEHOLDER(...)` line and the known simulation limits |
| AdvantageScope model | `ascope/`, made by `tools/step_to_glb.py` from the rev3 STEP |
| CAD package rev3 | `C:\Users\klape\Downloads\robotA_cad_package_rev3_2026-11-23.zip` (not in the repo) |

## Build and test (on Kevin's Windows PC)

- Set `JAVA_HOME` to WPILib's JDK, `C:\Users\Public\wpilib\2026\jdk`; there is no other Java on the PATH.
- VS Code's Java extension holds directories under `build/` open, so a recompiling Gradle run fails with
  "Unable to delete directory". Run Gradle with an init script that moves the build directory out of
  the repository (`allprojects { layout.buildDirectory = new File("<elsewhere>") }`), or close VS Code.
- `./gradlew build` compiles and runs the unit tests (`test`, about 15 s).
- `./gradlew simTest` runs the simulation tests (about 12 minutes; Phoenix's simulated devices run in real
  time, so keep the PC quiet). Each test class runs in its own JVM (`forkEvery = 1`).
- `./gradlew simulateJava` starts the robot simulation; add `-PheadlessSim` with no display.

## Done

| Task | What exists |
|---|---|
| 1 to 17, 19 | The whole robot as designed on November 21: drivetrain, elevator, wrist, rollers, claw, vision, superstructure, scoring, controls, game pieces, autos, docs |
| 18 | AdvantageScope model (7 motion groups) and layout |
| rev3 CAD | CAN map and ports, wrist encoder on the roboRIO, six-band keep-out with the unstow band, climber with pawls and funnel latch, masses, loads, limits, bumpers and setpoints |

In simulation every auto scores exactly its branches on both alliances.

## Decisions that override the plan's text

1. **Planner, algae.** The keep-out table outranks the "+10 degrees with algae" rule. STOW with an algae maps to `ALGAE_CARRY`.
2. **Planner, judging.** Measurements are clamped to the soft limits; a NaN measurement throws. The measured wrist is judged 1 degree worse than it reads. `next()` takes both velocities: the wrist target must be legal over every zone the carriage passes before it can stop (one loop, still accelerating, plus Motion Magic deceleration, plus 0.25 in either side), and the carriage only moves where the wrist's whole stopping sweep is legal.
3. **Planner, path.** The wrist heads at once for an angle legal in every band between the carriage and its goal.
4. **Unstow band.** `KeepOut` allows any angle from 0.75 to 2.0 in; carriage moves in and out of stow stop at 1.25. The drawing's table has no such band (see open-numbers for how it was measured). A carriage that dips below it with the wrist out rises to 1.25 with the wrist held.
5. **Reef routing.** Via points on a 2.0 m circle; same-face legs go direct and may fail `pathClear`.
6. **Drive-to-point.** Speed is `min(max(0, PD) + friction, maxSpeed)`; the derivative resets on a new target.
7. **Constants.** Dimensions and ratios in `RobotConstants`; gains and thresholds in the class that uses them, marked `PLACEHOLDER(tuning)`.
8. **Elevator.** Phoenix in rotor rotations; 110 in/s; the profile asks for 0.55 g under the 0.6 g cap. A start-up reading above 1 in is a zero kept from before a code restart and is kept without moving. A motor-controller reboot while running marks the position lost: no motion, an alert, and only a code restart clears it.
9. **L4 and the net** stay at the 65.25 in soft limit, not 66 (the up-stops). L4 at -52 degrees points at the same branch tip as -53 at 66; the net loses 0.75 in of its 3.9 in margin.
10. **Sim harness.** Three physics steps per 20 ms tick. Robot-level test classes share one robot per class (`testutil/RobotFixture`) because Phoenix's simulator slows as devices are created and closed.
11. **Wrist.** Closes its loop on its own rotor, seeded from the duty-cycle encoder. Trusted only while the encoder is connected, plausible, and within 5 degrees of the motor (debounced 0.1 s).
12. **Untrusted wrist** freezes the carriage where it is (not "at or below 1.0 in").
13. **Release.** Starts only at the pose, then runs until cleared; a score finishes only once the release has really started, and with the sensor offline it pushes for the full 1.0 s.
14. **Sensors offline.** `hasCoral()`/`hasAlgae()` read absent; the superstructure keeps the last known piece for its interlocks until a long enough push out. An auto scoring leg runs blind when the coral sensor is offline instead of skipping.
15. **Drive.** Every drive request goes through `FieldSpeedsRequest` so FOC follows `PRO_LICENSED`. The drive's acceleration is held to 80 % of the tipping acceleration on drawing R-001 at the present carriage height. Drive motors are limited to 60 A stator (EL-001's starting value).
16. **Autos.** An auto started without the start-pose button raises an alert and puts the estimate on its start pose. The button does nothing while enabled.
17. **Simulated bus load.** Mechanism IO classes call `optimizeBusUtilization`; the simulated drivetrain runs odometry at 100 Hz (the real robot keeps 250 Hz).
18. **Climb.** D-pad right, held: the CAD's sequence with one change, pawls down as soon as the fork is level instead of after the cage seats. A second hold winds in; D-pad left lets an empty fork back down. The funnel needs the button held 0.7 s in the climb pose, and on a field the last 30 s. An algae aboard is dropped first. The pawls lift only once the rope has held still with next to no current, so a hanging robot cannot be let down. Once the funnel fires the mechanism stays in the climb pose, across code restarts (`CommitMemory`, on the roboRIO's disk), until the pit reset or the next match with the fork stowed. A restart that cannot tell where the fork is (a motor controller that lost power) stops the winch for good.
19. **A pull under load** goes on through a wrist fault, and may be restarted with the wrist sensor dead; it may not be restarted with the wrist known to be out of the pose. A wrist fault before the first pull blocks the climb. That last is a choice the drive team may want to change.

## Open problems

1. **The design doc is behind the code and the CAD:** no climber section, the old CAN map, keep-out table and setpoints. `docs/cad` and this file are the current word.
2. **Only the robot can show:** servo pulses and directions, climber and wrist motor directions, encoder offset and sign, that the Talons keep their position across a code restart, the unstow band, tray opening and pawl lifting times, a real cage. All are marked `PLACEHOLDER(robot-check)` or listed in the README's "Set up the robot".
3. **Weight:** 115.6 lb against the 115 lb limit, on the CAD's estimate.
4. **L1** (5 in, -35 degrees) is not on the CAD's setpoint list.

## Minor review findings still open

- With the fork out, a disable lets the wrist droop and the carriage sag, and the next enable drives them back to the climb pose. The CAD rule says no motion; leaving them sagged into the bay seemed worse.
- `ClimbMatchTest` schedules the commands directly; the controller bindings themselves are not tested.
- `DeviceCheck` cannot report the PDH missing (constructing `PowerDistribution` does not fail), and runs only at boot.
- A lost elevator leader (`connected` false) leaves a stale height feeding the planner.
- Vision applies fixes with no latency compensation (about 7 cm at 3 m/s).
- A score ends about 0.25 s after the coral sensor clears; check on the robot that the coral is clear by then.
- simulateJava threading: `VisionIOSim` reads the true pose from the main thread while the 5 ms notifier steps the world; some subsystem fields are read across threads without `volatile` (simulation only).
- `ElevatorIOTalonFX` does not call `optimizeBusUtilization`.
- A home switch stuck closed zeroes wherever the carriage is.
- A NaN goal passes through the planner; `MARGIN_IN` equals `CARRIAGE_TOL_IN`; the reef-centre test is not independent; a misleading comment in `CameraModel`; no test for `dtSeconds <= 0`; `ElevatorSimTest` keeps its own alert helper.
