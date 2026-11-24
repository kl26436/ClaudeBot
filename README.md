# ClaudeBot

Robot code for Team 4206's ClaudeBot: swerve drive, a three-stage elevator, a wrist with coral rollers
and an algae claw, a winch climber, and one Limelight 4. It runs the real robot and a physics simulation of it from
the same code. The design is in [docs/superpowers/specs](docs/superpowers/specs/2026-11-21-claudebot-code-design.md),
the final CAD's numbers and the climb in [docs/cad](docs/cad/SOFTWARE_HANDOFF.md),
the CAN and power map in [docs/wiring.md](docs/wiring.md), and every unconfirmed value in
[docs/open-numbers.md](docs/open-numbers.md).

Java 17, WPILib 2026 (GradleRIO 2026.2.1), AdvantageKit 26, Phoenix 6 26.3 with Pro features on,
and maple-sim 0.4.0-beta for the 2025 field.

## Build

Install WPILib 2026; it brings the JDK. From the project folder:

```
./gradlew build          # compile and run the unit tests (Windows: .\gradlew build)
./gradlew deploy         # to the robot, over USB or the robot network
```

If Gradle reports that `JAVA_HOME` is not set, point it at WPILib's JDK, for example
`C:\Users\Public\wpilib\2026\jdk` on Windows.

## Run the simulator

```
./gradlew simulateJava                 # with the WPILib simulation window
./gradlew simulateJava -PheadlessSim   # without it, for a machine with no display
```

Plug in an Xbox controller and enable teleop from the simulation window. Controls are in the
design's Driver controls table: left stick drives field-relative, right stick turns, Y/X/B/A pick
L4 to L1, the triggers align and score on the nearest reef face (right or left branch), right bumper
picks up at the nearest station, left bumper takes reef algae, D-pad up scores the net, D-pad down the
processor, Back stows and cancels, Start resets the heading.

Hold D-pad right to climb. The first hold moves the mechanism out of the way, opens the funnel and lets
the fork down to level; let go, back onto the cage, and hold it again to wind in. Letting go pauses at any
point. After a missed cage, hold D-pad left to let the empty fork back down; it refuses if the robot is
hanging on it. Opening the funnel cannot be undone in a match: from then on the mechanism stays in the climb
pose. On a field the climb is locked until the last 30 s.

**After every climb, in the pit:** with the robot disabled, stow the fork, latch the tray, and press
`Climber/Pit reset` on the dashboard. The robot remembers a climb across code restarts, because nothing on it
can tell a hanging fork from a stowed one. If the reset is forgotten, the next match on a field clears it as
long as the fork is in stow; in practice nothing does.

The simulation steps every mechanism against a physics model and the field with maple-sim. A coral
arrives 0.7 s after the robot settles at a station with the intake running, and each released piece
is judged from the robot's true position, so the logged score is what would really have happened.

## Watch it in AdvantageScope

Connect AdvantageScope 2026 to the simulator (or open a log from `logs/`). Useful keys:

| Key | What it is |
| --- | --- |
| `Sim/TruePose` | Where the robot really is |
| `Drive` inputs (pose, module states) | What odometry thinks |
| `Mechanism/Components` | Stage A, stage B, carriage, wrist, climber fork and funnel tray, for the robot model |
| `Climber/*` | Step of the climb, fork angle, pawls |
| `Sim/Coral`, `Sim/Algae` | Game pieces on the robot and the field |
| `Superstructure/*` | Wanted state, targets, at-pose |
| `Auto/Selected`, `Auto/PoseSet` | The chosen auto and whether its start pose was set |

The CAD robot model and a layout are in `ascope/`. In AdvantageScope, choose Use Custom Assets
Folder from the menu and pick `ascope/`; then Import Layout and pick `ascope/layout.json`. Set the live source to
NetworkTables 4 (AdvantageKit) so the keys match the layout's. The layout opens the 3D field (true pose with
the CAD model, odometry as a green ghost), the swerve states, and target against measured for the carriage,
wrist and climber.

`tools/step_to_glb.py` makes the model from the CAD package's `robotA_assembly_complete.step`, one file for
each group that moves together.

## Tests

```
./gradlew test      # layer 1: rules and geometry, no simulator, about 5 s
./gradlew simTest   # layers 2 and 3: mechanisms, teleop cycles and every auto, in physics
```

`simTest` takes 10 to 15 minutes. Phoenix's simulated devices run in real time, so it cannot be sped
up, and each test class runs in its own JVM. Run one class with
`./gradlew simTest --tests '*AutoMatchTest'`.

On Windows, VS Code's Java extension can hold files under `build/` open and make Gradle fail with
"Unable to delete directory". Close the editor or run Gradle with a different build directory.

## Set up the robot

- **CAN:** IDs and ports in [docs/wiring.md](docs/wiring.md). Name the CANivore `canivore`.
- **Limelight 4:** name `limelight`, an AprilTag pipeline, 1280 x 800, with corner output on.
- **Wrist encoder:** with the wrist held level, read `Wrist/AbsoluteDeg` and set `ENCODER_OFFSET_ROT` (and
  `ENCODER_SIGN` if the reading falls as the tip rises) in `WristIOTalonFX`.
- **Servos:** fit each horn so 1500 us is the middle of its swing, then find the pulse at each end
  (`ClimberIOTalonFX`). Do the pawls with the rope off.
- **Start every match with the fork stowed:** nothing measures it, and its angle counts from there.
- **Unstow band:** before trusting it, move the wrist through its range by hand with the carriage at 0.75,
  1.25 and 2.0 in.

## Add an auto

1. Add a value to `autos/Auto.java`.
2. Add a package-private method to `autos/AutoFactory.java` that returns an `AutoRoutine`: its start
   pose (or `Optional.empty()` for anywhere), the command, and the branches it should score on. Build
   it from the existing legs (`scoreLeg`, `stationLeg`) so every leg keeps its timeout.
3. Add one row to `AutoChooser.PROGRAMS` with the label the dashboard should show.
4. Run `./gradlew simTest --tests '*AutoMatchTest'`. The new auto runs on both alliances and must
   score exactly the branches it lists, within 15 s, without breaking a keep-out rule.

Before a match: pick the auto on the dashboard (`SmartDashboard/Auto/Chooser`), place the robot on
its start spot, and press `Auto/Set pose to auto start`. `Auto/PoseSet` goes false whenever the
selection or the alliance changes.

## Layout

| Folder | Holds |
| --- | --- |
| `config/` | `Ports` (every CAN ID and port), `RobotConstants` (dimensions, ratios), `Setpoints`, `KeepOut` |
| `subsystems/` | One folder per mechanism (IO interface, TalonFX class, simulation subclass, subsystem), plus `Superstructure` and its pure planner |
| `commands/` | Scoring and pickup moves shared by teleop and autos, and the start-up device check |
| `autos/` | The auto list, chooser and factory |
| `field/` | Reef, station, barge and start poses for both alliances |
| `sim/` | The simulated robot and the score judge |

## The playbook

[Team 4206 Playbook](https://kl26436.github.io/FRC-Playbook/) tells the whole story of this robot, from kickoff to code. The
reasons and the math behind the code are in [docs/playbook](docs/playbook/code-decisions-and-math.md), with replays of
every simulated auto. Regenerate the replays with `./gradlew simTest --tests '*AutoMatchTest*' -PreplayDir=docs/playbook/replays`.

## Credits

ClaudeBot builds on work other teams and projects share in the open. Thank you to:

- **[FRC Team 2910, Jack in the Bot](https://github.com/FRCTeam2910).** The project follows their layout. The autos
  (chooser, programs, legs) are adapted from their
  [2025 robot code](https://github.com/FRCTeam2910/2025CompetitionRobot-Public), and the swerve drive and its simulation
  from their [2026 robot code](https://github.com/FRCTeam2910/2026CompetitionRobot-Public). Drive-to-point and the
  single-tag position fix use their methods, written fresh here.
- **[FRC Team 5516, Iron Maple](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)** and Shenzhen Robotics Alliance, for
  maple-sim, the field physics in the simulator.
- **[FRC Team 6328, Mechanical Advantage](https://github.com/Mechanical-Advantage)** (Littleton Robotics), for AdvantageKit and
  AdvantageScope. The logging setup comes from AdvantageKit's templates.
- **[WPILib](https://github.com/wpilibsuite)**, for the libraries, the simulator and the project template.
- **[CTR Electronics](https://github.com/CrossTheRoadElec)**, for Phoenix 6. `TunerConstants` follows the shape Tuner X generates.

The code was written with [Claude Code](https://claude.com/claude-code).

## License

[MIT](LICENSE), except for the parts that belong to others. [NOTICE](NOTICE) lists those file by file, with the full texts in
[licenses/](licenses). Manufacturers' geometry in the 3D robot model is not covered by the MIT license.
