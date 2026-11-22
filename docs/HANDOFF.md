# Robot A code: handoff

Updated 2026-11-22. Read this before the plan. It records where the build stands, the decisions that
override the plan's text, and what is still open.

## Prompt to start the next session

Paste this into a new session opened on this folder:

> Continue the Robot A robot code in this repository. Read `docs/HANDOFF.md` first, then the plan at
> `docs/superpowers/plans/2026-11-21-robot-a-code.md`. Tasks 1 to 17 and 19 are done, committed and
> reviewed. Do not start Task 18 or the climber until I say the CAD corrections listed under Open
> problems are in. Do not push to any remote. Ask me before changing `Ports.java` or the keep-out table.

## Where things are

| Thing | Location |
|---|---|
| Code | this repository, branch `robot-a`. Nothing has been pushed anywhere. |
| Plan | `docs/superpowers/plans/2026-11-21-robot-a-code.md` (19 tasks) |
| Design | `docs/superpowers/specs/2026-11-21-robot-a-code-design.md`, exported from the Claude Doc "Robot A code design" (https://claude.ai/artifact/6YYpCCkQaGtHnapUWu4cRd, rev 36) |
| Wiring | `docs/wiring.md` (the design's CAN and power tables) |
| Unconfirmed values | `docs/open-numbers.md`: every `PLACEHOLDER(...)` line (82) and the known simulation limits |
| CAD package rev2 | `C:\Users\klape\Downloads\robotA_cad_package_rev2_2026-11-21.zip` (not in the repo; see Open problems) |

## Build and test (on Kevin's Windows PC)

- Set `JAVA_HOME` to WPILib's JDK, `C:\Users\Public\wpilib\2026\jdk`; there is no other Java on the PATH.
- VS Code's Java extension holds directories under `build/` open, so a recompiling Gradle run fails with
  "Unable to delete directory". Run Gradle with an init script that moves the build directory out of
  the repository (`allprojects { layout.buildDirectory = new File("<elsewhere>") }`), or close VS Code.
- `./gradlew build` compiles and runs the 76 unit tests (`test`, layer 1, about 10 s).
- `./gradlew simTest` runs the 121 simulation tests tagged `sim` (about 11.5 minutes; Phoenix's simulated
  devices run in real time). Each test class runs in its own JVM (`forkEvery = 1`).
- `./gradlew simulateJava` starts the robot simulation; add `-PheadlessSim` with no display.

## Done

| Task | What exists |
|---|---|
| 1 to 8 | Scaffold, `Ports`, constants, keep-out table, planner, field geometry, single-tag fix, drive-to-point |
| 9, 10 | Elevator and wrist with physics |
| 11 | Coral rollers and algae claw, with `PieceSensor` |
| 12 | `Superstructure` |
| 13 | CTRE swerve behind `SwerveIO`, maple-sim field, `RobotState` |
| 14 | Vision (`VisionIOLimelight`, `VisionIOSim`, `Vision`) |
| 15 | `ScoreCommands`, driver bindings in `RobotContainer`, `DeviceCheck`, `SimulatedRobot` |
| 16 | `ScoreJudge`, game pieces, `Barge` |
| 17 | Autos: `Auto`, `AutoProgram`, `AutoRoutine`, `AutoFactory`, `AutoChooser` |
| 19 | `docs/wiring.md`, `README.md`, open numbers, the design export |

One independent review of the whole branch was run after Task 19. Its critical finding and all five
important findings are fixed (commits after `0f3943f`). Its minor findings are listed at the end.

In simulation every auto scores exactly its branches on both alliances; the three-coral autos land the
third coral at about 14.0 s.

## Decisions that override the plan's text

1. **Planner, algae.** The keep-out table outranks the "+10 degrees with algae" rule. STOW with an algae maps to `ALGAE_CARRY`.
2. **Planner, judging.** Measurements are clamped to the soft limits; a NaN measurement throws. The measured wrist is judged 1 degree worse than it reads. Since `2f80fc3` `next()` also takes both velocities: the wrist target must be legal over every zone the carriage passes before it can stop (one loop plus Motion Magic deceleration), and the carriage only moves where the wrist's whole stopping sweep is legal.
3. **Reef routing.** Via points on a 2.0 m circle; same-face legs go direct and may fail `pathClear`.
4. **Drive-to-point.** Speed is `min(max(0, PD) + friction, maxSpeed)`; the derivative resets on a new target.
5. **Constants.** Dimensions and ratios in `RobotConstants`; gains and thresholds in the class that uses them, marked `PLACEHOLDER(tuning)`.
6. **Elevator.** Cruise 65 in/s, not 110 (Open problem 1). A start-up reading above 1 in is a zero kept from before a code restart and is kept without moving. A motor-controller reboot while running marks the position lost: no motion, an alert, and only a code restart clears it.
7. **Sim harness.** Three physics steps per 20 ms tick. Robot-level test classes share one robot per class (`testutil/RobotFixture`) because Phoenix's simulator slows as devices are created and closed.
8. **Wrist.** Trusted only while the CANdi's PWM period is valid, the angle is plausible, and the motor and encoder agree within 5 degrees (debounced 0.1 s). The CANdi's PWM frames run at 250 Hz.
9. **Untrusted wrist** freezes the carriage where it is (not "at or below 1.0 in").
10. **Release.** Starts only at the pose, then runs until cleared; a score finishes only once the release has really started, and with the sensor offline it pushes for the full 1.0 s.
11. **Sensors offline.** `hasCoral()`/`hasAlgae()` read absent; the superstructure keeps the last known piece for its interlocks until a long enough push out. An auto scoring leg runs blind when the coral sensor is offline instead of skipping.
12. **Drive.** Every drive request goes through `FieldSpeedsRequest` so FOC follows `PRO_LICENSED`. The drive's acceleration is held to 80 % of the tipping acceleration on drawing R-001 at the present carriage height. The first drive test targets branch H, not A (A is behind the reef from the centre start).
13. **Autos.** An auto started without the start-pose button raises an alert and puts the estimate on its start pose. The button does nothing while enabled.
14. **Simulated bus load.** Mechanism IO classes call `optimizeBusUtilization`; the simulated drivetrain runs odometry at 100 Hz (the real robot keeps 250 Hz).

## Open problems

1. **Elevator cruise speed in simulation** (unchanged). At 110 in/s the Motion Magic reference stops decelerating properly and the carriage hits the top. First thing to try: keep Phoenix in rotor rotations (`SensorToMechanismRatio = 1`) and convert in the IO class. Do not raise the speed on the robot near the top of travel until this is understood.
2. **CAD rev2 disagrees with the approved design.** Feedback sent to the design agent on 2026-11-22. Waiting on it:
   - the CAN map (bus name `canivore` against `drive`; Pigeon 10 against 2; most mechanism IDs shifted; climber on 57/58; CANranges on 61/62);
   - the wrist encoder wired to roboRIO DIO 1 instead of the CANdi the design (decision 1) and the code use;
   - the six-zone keep-out table still has no height where the wrist may be between -90 and -75 (no way out of stow);
   - intake drawn at carriage 1.25 in, exactly on a zone boundary;
   - breaker sizes for the moving harness (all 40 A in rev2, 30 A with 14 AWG in the design);
   - the funnel latch servo, climber motors and pawl servos, which are not in the design yet.
   Until it is settled, `Ports` and the four-zone keep-out table stay as they are.
3. **Values to take from the CAD once it settles:** rollers and claw are Kraken X44 (the sim models X60); robot mass 146.45 lb on the field (code 140); bumper face 17.25 in (code 17.4); stage A and B weights; wrist inertia; Limelight position.
4. **Task 18** (AdvantageScope model) can use `step/robotA_assembly_complete.step` from rev2, but should wait for the CAD corrections.
5. **Climber**: needs a design-doc section and sign-off before any code (the CAD's climb sequence and interlocks are on drawing R-001).

## Minor review findings still open

- `DeviceCheck` cannot report the PDH missing (constructing `PowerDistribution` does not fail), and runs only at boot.
- A lost elevator leader (`connected` false) leaves a stale height feeding the planner; the wrist ignores `inputs.connected` (the agreement check covers most of it).
- Vision applies fixes with no latency compensation (about 7 cm at 3 m/s); the README does not yet say how to set up the Limelight (name `limelight`, AprilTag pipeline, corner output, 1280 x 800) or that the CANivore must be named `drive`.
- A score ends about 0.25 s after the coral sensor clears; check on the robot that the coral is clear by then.
- simulateJava threading: `VisionIOSim` reads the true pose from the main thread while the 5 ms notifier steps the world; some subsystem fields are read across threads without `volatile` (simulation only).
- `ElevatorIOTalonFX` does not call `optimizeBusUtilization` and relies on default current limits.
- A home switch stuck closed zeroes wherever the carriage is.
- From the first handoff: a NaN goal passes through the planner; `MARGIN_IN` equals `CARRIAGE_TOL_IN`; the reef-centre test is not independent; a misleading comment in `CameraModel`; no test for `dtSeconds <= 0`; `ElevatorSimTest` keeps its own alert helper.
- README omits D-pad down (processor); a test comment calls positive pitch "nose-up".
