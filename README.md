# Robot A

Robot code for Team 4206's Robot A: swerve drive, a three-stage elevator, a wrist with coral rollers
and an algae claw, and one Limelight 4. It runs the real robot and a physics simulation of it from
the same code. The design is in [docs/superpowers/specs](docs/superpowers/specs/2026-11-21-robot-a-code-design.md),
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
picks up at the nearest station, left bumper takes reef algae, D-pad up scores the net, Back stows
and cancels, Start resets the heading.

The simulation steps every mechanism against a physics model and the field with maple-sim. A coral
arrives 0.7 s after the robot settles at a station with the intake running, and each released piece
is judged from the robot's true position, so the logged score is what would really have happened.

## Watch it in AdvantageScope

Connect AdvantageScope 2026 to the simulator (or open a log from `logs/`). Useful keys:

| Key | What it is |
| --- | --- |
| `Sim/TruePose` | Where the robot really is |
| `Drive` inputs (pose, module states) | What odometry thinks |
| `Mechanism/Components` | Stage A, stage B, carriage and wrist poses, for the robot model |
| `Sim/Coral`, `Sim/Algae` | Game pieces on the robot and the field |
| `Superstructure/*` | Wanted state, targets, at-pose |
| `Auto/Selected`, `Auto/PoseSet` | The chosen auto and whether its start pose was set |

The CAD robot model and a saved layout will live in `ascope/`. That step (plan Task 18) is waiting
for the final CAD STEP files; until then use AdvantageScope's default robot with
`Mechanism/Components` as its components. Once `ascope/` exists, choose Use Custom Assets Folder in
AdvantageScope and pick it.

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

Files adapted from Team 2910's public code carry a notice header; [NOTICE](NOTICE) has the licence.
