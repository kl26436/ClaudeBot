# Robot A code: handoff

Written 2026-11-21 at commit `29b5e64` plus this file. Read this before the plan. It records where the
build stopped, the decisions that override the plan's text, and what each remaining task needs to know.

## Prompt to start the next session

Paste this into a new session opened on this folder:

> Continue building the Robot A robot code in this repository. Read `docs/HANDOFF.md` first, then the
> plan at `docs/superpowers/plans/2026-11-21-robot-a-code.md`. Tasks 1 to 10 are done and committed.
> Start at Task 11 and work through Task 17 and Task 19 in order, yourself, in this session: no
> subagent per task. For each task write the tests first, watch them fail, implement, run
> `./gradlew build simTest`, and commit with the message the plan gives. Where this handoff and the plan
> disagree, the handoff wins. Do not start Task 18 or the climber until I give you the CAD files.
> Do not push to any remote. When every task is done, run one independent review of the whole branch
> and report its findings to me before fixing anything.

## Where things are

| Thing | Location |
|---|---|
| Code | this repository, branch `robot-a`. Nothing has been pushed anywhere. |
| Plan | `docs/superpowers/plans/2026-11-21-robot-a-code.md` (19 tasks, 4 milestones) |
| Design | Claude Doc "Robot A code design", https://claude.ai/artifact/6YYpCCkQaGtHnapUWu4cRd (approved 2026-11-21) |
| Unconfirmed values | `docs/open-numbers.md`; find them with `grep -rn "PLACEHOLDER(" src` |
| Team 2910 reference code | public MIT repos (2025 and 2026 robot code). Not vendored here. |

## Build and test

- `./gradlew build` compiles and runs the unit tests (59, about 5 s). On Windows use `.\gradlew`.
- `./gradlew simTest` runs the physics tests tagged `sim` (35, about 90 s). They run in real time because
  Phoenix's simulated devices do, so they cannot be sped up. Run one class with
  `./gradlew simTest --tests '*WristSimTest'`.
- `./gradlew simulateJava` starts the full robot simulation; add `-PheadlessSim` on a machine with no display.
- Stack: Java 17, WPILib 2026 (GradleRIO 2026.2.1), AdvantageKit 26.0.2, Phoenix 6 26.3.0, maple-sim
  0.4.0-beta (the published `0.4.0-beta-obstacles-fix` does not resolve; stay on `0.4.0-beta`), JUnit 5.

## Done (tasks 1 to 10)

| Task | What exists | Tests |
|---|---|---|
| 1 Scaffold | Gradle project, vendordeps, `Robot`, `RobotContainer`, `test` and `simTest` tasks | builds, sim starts |
| 2 CAN map | `config/Ports` | `PortsTest` |
| 3 Constants | `config/RobotConstants`, `config/Setpoints` | `UnitsTest` |
| 4 Keep-out table | `config/KeepOut` | `KeepOutTest` |
| 5 Planner | `subsystems/SuperstructurePlanner` | `SuperstructurePlannerTest` |
| 6 Field geometry | `field/Reef`, `Station`, `StartPose`, `AllianceFlip` | `ReefTest`, `StationTest` |
| 7 Single-tag fix | `subsystems/vision/SingleTagFix`, `CameraModel`, `TagObservation` | `SingleTagFixTest` |
| 8 Drive-to-point | `subsystems/drive/DriveToPoint` | `DriveToPointTest` |
| 9 Elevator | `subsystems/elevator/*`, `testutil/SimHarness` | `ElevatorSimTest` (17, sim) |
| 10 Wrist | `subsystems/wrist/*`, `testutil/AlertProbe` | `WristSimTest` (18, sim) |

Tasks 1 to 8 each had an independent review. Tasks 9 and 10 were checked by the author only.

## Decisions that override the plan's text

1. **Planner, algae.** The keep-out table outranks the "wrist at or above +10° with algae" rule: the floor
   applies only where the current zone's wrist range reaches +10°. With algae and a STOW goal the planner
   parks at (1.75 in, +10°) and never reports the goal reached. **Task 12 must map a stow request while
   holding algae to `ALGAE_CARRY`.**
2. **Planner, joint targets.** The carriage target is clamped into the span legal for both the measured
   wrist and the commanded wrist target. Measurements are clamped to the soft limits first; a NaN
   measurement throws `IllegalArgumentException`.
3. **Reef routing (Task 6).** Via points are on a 2.0 m circle at the six corner bearings (30° + 60°·k),
   not 1.75 m. `pathClear` is false only when the segment's closest point to the reef centre is strictly
   inside the segment and nearer than 1.69 m, so leaving from or arriving at a scoring pose is clear.
   `route` goes direct when both ends are on the same reef face. **Tasks 15 and 17: do not assume every
   leg of a route passes `pathClear`** (same-face legs may not).
4. **Drive-to-point (Task 8).** Speed is `min(max(0, PD) + friction, maxSpeed)`, not `|PD|`. The derivative
   resets when the target changes.
5. **Constants.** Physical dimensions and gear ratios live in `RobotConstants`. Gains, tolerances and
   algorithm thresholds live in the class that uses them, each gain marked `// PLACEHOLDER(tuning)`.
6. **Elevator (Task 9).** `CRUISE_IN_PER_S` is 65, not 110 (see Open problems). Soft limits are off until
   the elevator is zeroed and while homing volts are applied. `Elevator` and the IO classes have `close()`.
   A kA gain was added.
7. **Sim harness.** `SimHarness` has `reset()`, `DT` and `lastDtSeconds()` beyond the plan. It runs the
   registered physics steps three times per 20 ms tick, 5 ms apart, then waits 6 ms before the scheduler
   reads signals. One physics step per tick made Phoenix's 1 kHz loops see a sawtooth and draw fake current.
8. **Wrist (Task 10).** "Encoder connected" means the CANdi's PWM1 rise-to-rise period is 500 to 2000
   microseconds (Phoenix reports that signal in microseconds; the CANdi has no connected flag). The wrist
   holds in brake until its first goal. The alert is error level. `WristIOSim.trueWristDeg()` was added.

## Patterns the remaining tasks must follow

- **Mechanism shape:** `XxxIO` (interface with `@AutoLog` inputs), `XxxIOTalonFX` (real devices, used on the
  robot and in sim), `XxxIOSim extends XxxIOTalonFX` (feeds Phoenix sim states from a WPILib physics
  model), `Xxx extends SubsystemBase`. Copy the wrist's files as the template.
- **Sim test fixture** (see `WristSimTest.setUp`): `SimHarness.reset()`; build the sim IO and subsystem;
  `SimHarness.enable()`; `SimHarness.run(0.06)`; only then
  `SimHarness.register(() -> sim.simulate(SimHarness.lastDtSeconds()))`; `SimHarness.run(0.14)`.
  The simulated controllers outlive each test and keep the last test's request; the first three ticks let
  the new subsystem replace it before the physics can respond. Call `SimHarness.reset()` and the
  `close()` methods in `@AfterEach`.
- **Alerts in tests:** `AlertProbe.active("text")`.
- **Assert on the truth, not only the sensor.** Sim IO classes expose the physics state
  (`trueWristDeg()`); assert on it as well as on the subsystem's reading.
- **Phoenix facts found the hard way:** Phoenix simulates the CANdi (`CANdiSimState.setPwm1Position`,
  `setPwm1Connected`, `setPwm1RiseRise`). Check with `javap` on the Phoenix jar whether it simulates the
  CANrange before Task 11; if not, feed the range at the IO layer and note it in `docs/open-numbers.md`.
  `StatusSignal.getValueAsDouble()` returns the signal's own unit, which is not always SI.
- **Tests:** every test has real assertions; where the plan shows a comment for a test body, write the
  assertions the comment describes. Never weaken a limit to make a test pass.
- **Commits:** one per task with the plan's message, plus fix commits. Nothing is pushed.

## Remaining tasks

| Task | Carry-in notes |
|---|---|
| 11 Coral rollers and algae claw | Follow the patterns above. Claw stator limit test needs the three-step harness (already in place). |
| 12 Superstructure | Decision 1 (algae + stow goes to `ALGAE_CARRY`). The planner throws on NaN, so guard untrusted wrist readings before calling it. An untrusted wrist or un-homed elevator must hold the other axis still. |
| 13 Drivetrain and `RobotState` | CTRE swerve API behind an IO interface, on the `drive` CAN bus. No time-synced signals or FOC when `PRO_LICENSED` is false. |
| 14 Vision | Reject a tag fix that is non-finite, or whose `heightPx` is zero or tiny, before it reaches the pose estimate (`SingleTagFix.distanceM` does not guard this). |
| 15 Scoring commands, controls, device check | Decision 3. |
| 16 Game pieces and score judge | maple-sim arena `seasonspecific.reefscape2025`. |
| 17 Autos | Decision 3. Start poses are `PLACEHOLDER(field-check)`. |
| 18 AdvantageScope model | **Blocked** until the CAD STEP files are final. |
| 19 Wiring doc, README, full run | Also export the design doc into `docs/superpowers/specs/`. |

After Task 19: one independent whole-branch review (the plan's Review Focus section lists five failure
modes to check), then decide with Kevin how to finish the branch.

## Open problems

1. **Elevator cruise speed in simulation.** At the design speed of 110 in/s the carriage hits the top hard
   stop. A trace at 110 in/s shows the Motion Magic *reference* is at fault, not the physics: its velocity
   drops from 110 to about 100 in/s at the start of the slow-down and then barely falls (about 12 in/s²
   instead of 231.65) until the reference reaches the target. Below about 70 in/s the profile is normal.
   It does not depend on gains, FOC, soft limits or the physics step rate. Not yet tried: keeping Phoenix
   in rotor rotations (`SensorToMechanismRatio = 1`) and converting to inches in the IO class, which is
   the first thing to test. Until this is understood, do not raise the cruise speed on a real robot near
   the top of travel.
2. **CAD package.** Still needed from Kevin: `robotA_can_map.csv`, the README and the layout and weight
   drawings. The climber (two rope winches), the funnel servo latch and the wrist climb pose are not in
   the design or the plan. They need a design update and Kevin's sign-off before any code.
3. **Unstow gap in the new keep-out table.** The CAD's six-zone table (below 1.25 in: stow only; 1.25 to
   7: at or above -75°; 7 to 9: -60°; 9 to 15: -45°; 15 to 23: -40°; above 23: any) leaves no carriage
   height at which the wrist can turn out of stow. The code still uses the four-zone table with a
   `PLACEHOLDER(funnel-redesign)` limit of -38° from 6 to 23 in. Do not switch tables until this is resolved.

## Small findings left for the final review

- Planner: a NaN goal passes through as a NaN setpoint. `MARGIN_IN` equals `CARRIAGE_TOL_IN`, so a future
  pose within 0.5 in of a blocked edge would never finish.
- Reef: the test computes the reef centre the same way the code does, so nothing checks it independently.
- Vision: a misleading "cosine test" comment in `CameraModel`; the noise path is untested.
- Drive-to-point: no test for `dtSeconds <= 0`.
- Elevator test class has its own `alertActive` helper; it could use `AlertProbe`.
