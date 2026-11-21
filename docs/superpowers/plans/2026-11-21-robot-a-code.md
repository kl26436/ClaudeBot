# Robot A Code Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One Java project that runs Robot A (swerve, elevator, wrist, coral rollers, algae claw, one Limelight 4) on the real robot and in a physics simulation shown in AdvantageScope, with selectable 2910-style autos and three layers of tests.

**Architecture:** Team 2910's layout. CTRE's swerve API sits behind an IO interface; each mechanism has one IO interface and one TalonFX class that both the robot and the simulator run. A simulation subclass adds only physics by feeding Phoenix's simulated motor from a WPILib model. A Superstructure coordinates elevator, wrist and rollers through a pure planner that keeps every move inside the CAD keep-out table. maple-sim supplies the 2025 field.

**Tech Stack:** Java 17, WPILib 2026 (GradleRIO), AdvantageKit 26, Phoenix 6 (26.x, Pro features on), maple-sim (2026 line, `seasonspecific.reefscape2025`), JUnit 5, AdvantageScope 2026.

**Spec:** [Robot A code design](https://claude.ai/artifact/6YYpCCkQaGtHnapUWu4cRd). Task 1 exports it to `docs/superpowers/specs/2026-11-21-robot-a-code-design.md`. Executors read both.

**Reference code (read-only, MIT):** `github.com/FRCTeam2910/2025CompetitionRobot-Public` (autos, drive-to-point, single-tag fix), `github.com/FRCTeam2910/2026CompetitionRobot-Public` (CTRE swerve + maple-sim wiring, 2026 library versions), `github.com/Shenzhen-Robotics-Alliance/maple-sim` (Reefscape arena API).

## Global Constraints

- Package `frc.robot`. Team number 4206. Java 17. Library versions pinned to the newest 2026 releases on the day Task 1 runs.
- Mechanism APIs use **inches of carriage travel** and **degrees from level (positive up, stow −90)**. Field geometry uses WPILib types in metres and radians.
- Every CAN ID, bus name and roboRIO port appears only in `config/Ports.java`. Every mechanism position appears only in `config/Setpoints.java` or `config/KeepOut.java`. Every dimension and ratio appears only in `config/RobotConstants.java`.
- A value that is not yet confirmed carries the comment `// PLACEHOLDER(<reason>)` on its line. Reasons in use: `funnel-redesign`, `drawing`, `tuning`, `field-check`.
- `RobotConstants.PRO_LICENSED = true`. When false, no code path may request FOC, a fused sensor or time-synced signals.
- No PathPlanner, Choreo or PhotonVision dependency.
- Files adapted from 2910 keep an MIT notice header; `NOTICE` at the repo root carries the full licence text.
- All logging goes through AdvantageKit `Logger`. IO inputs are `@AutoLog` classes.
- Tests: layer 1 (no HAL, no Phoenix) runs under `./gradlew test`. Layers 2 and 3 are tagged `@Tag("sim")` and run under `./gradlew simTest`. Phoenix's simulated devices run in real time, so `simTest` takes minutes; assertions use tolerances, never exact equality.
- The tag layout is `RobotConstants.FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded`. Switching to the AndyMark layout is that one constant.
- Test helpers named in a test but not listed in an Interfaces block (`homeThen`, `runRecording`, `runChecking`, `seatCoral`, `giveAlgae`, `placeAt`, `alertActive`, `schedule`, `AutoHarness` and the like) are private to that test class or live in `testutil/`. They do what their names say and nothing else.
- Commit after every task. Nothing is pushed to a remote.
- The climber, LEDs, a second controller and a pit self-test are out of scope.

## Review Focus

Failure modes the spec implies but does not test. Each has a test in the task named.

1. **Home switch never trips** (broken or unplugged). The elevator must stop homing after 3 s, zero on stall, raise an alert, and never run upward un-homed. Task 9.
2. **Wrist absolute encoder missing or implausible at start-up.** The wrist must not move and the elevator must stay below 1.25 in, with an alert. Task 10.
3. **A CANrange drops off the bus.** `hasCoral()` and `hasAlgae()` must report false with an alert, and every auto leg must end on its timeout instead of hanging. Tasks 11 and 17.
4. **Alliance unknown or changed after the auto was built.** Both alliances are built while disabled; an unknown alliance at auto start runs nothing and raises an alert. Task 17.
5. **Driver lets go mid-score with the elevator up.** The mechanism must return to stow or carry through legal poses, and the drive speed cap must hold until the carriage is under 23 in. Task 12.

---

## File Structure

```
build.gradle  settings.gradle  .wpilib/wpilib_preferences.json  vendordeps/  NOTICE  README.md
src/main/java/frc/robot/
  Main.java  Robot.java  RobotContainer.java  RobotState.java
  config/   Ports.java  RobotConstants.java  Setpoints.java  KeepOut.java
  field/    AllianceFlip.java  Reef.java  Station.java  StartPose.java
  subsystems/
    Superstructure.java  SuperstructurePlanner.java
    drive/    SwerveIO.java  SwerveIOCTRE.java  SwerveIOSim.java  SwerveSubsystem.java  DriveToPoint.java  TunerConstants.java
    elevator/ ElevatorIO.java  ElevatorIOTalonFX.java  ElevatorIOSim.java  Elevator.java
    wrist/    WristIO.java  WristIOTalonFX.java  WristIOSim.java  Wrist.java
    coral/    CoralRollersIO.java  CoralRollersIOTalonFX.java  CoralRollersIOSim.java  CoralRollers.java
    claw/     AlgaeClawIO.java  AlgaeClawIOTalonFX.java  AlgaeClawIOSim.java  AlgaeClaw.java
    vision/   VisionIO.java  VisionIOLimelight.java  VisionIOSim.java  Vision.java  SingleTagFix.java  CameraModel.java
  commands/ ScoreCommands.java  DeviceCheck.java
  autos/    Auto.java  AutoProgram.java  AutoChooser.java  AutoFactory.java
  sim/      SimulatedRobot.java  ScoreJudge.java
src/test/java/frc/robot/
  rules/     PortsTest  UnitsTest  KeepOutTest  SuperstructurePlannerTest  ReefTest  StationTest  SingleTagFixTest  DriveToPointTest
  mechanism/ ElevatorSimTest  WristSimTest  RollersSimTest  SuperstructureSimTest
  match/     DriveSimTest  VisionSimTest  TeleopCycleTest  AutoMatchTest
  testutil/  SimHarness.java
ascope/     Robot_RobotA/{model.glb, model_0..3.glb, config.json}  layout.json
tools/      step_to_glb.py
docs/       wiring.md  open-numbers.md  superpowers/{specs,plans}/
```

---

# Milestone 1: project and rules

Ends with a green `./gradlew build` and every layer 1 test passing.

### Task 1: Scaffold the project

**Files:**
- Create: `build.gradle`, `settings.gradle`, `.wpilib/wpilib_preferences.json`, `vendordeps/*.json`, `NOTICE`, `src/main/java/frc/robot/{Main,Robot,RobotContainer}.java`, `docs/superpowers/specs/2026-11-21-robot-a-code-design.md`, `docs/open-numbers.md`

**Interfaces:**
- Produces: `Robot extends LoggedRobot` with modes `REAL`, `SIM`, `REPLAY` chosen by `RobotConstants.currentMode()`; Gradle tasks `test` (excludes tag `sim`) and `simTest` (only tag `sim`, `maxParallelForks = 1`).

- [ ] **Step 1:** Start from AdvantageKit's 2026 skeleton template. Add vendordeps for Phoenix 6 and maple-sim. Set team 4206.
- [ ] **Step 2:** In `build.gradle` configure JUnit 5, `test { useJUnitPlatform { excludeTags 'sim' } }`, and a `simTest` task of type `Test` with `includeTags 'sim'`.
- [ ] **Step 3:** `Robot.java` sets up AdvantageKit: REAL logs to a USB stick and NetworkTables, SIM logs to NetworkTables and `logs/`, REPLAY reads a log and writes `_sim`.
- [ ] **Step 4:** Export the design doc to the spec path. Write `NOTICE` with 2910's MIT licence text. Create `docs/open-numbers.md` listing every `PLACEHOLDER` reason and what resolves it.
- [ ] **Step 5:** Run `./gradlew build`. Expected: `BUILD SUCCESSFUL`.
- [ ] **Step 6:** Run `./gradlew simulateJava`. Expected: the robot program starts and prints the AdvantageKit banner with no exceptions.
- [ ] **Step 7:** Commit `chore: scaffold Robot A project`.

### Task 2: CAN map

**Files:**
- Create: `config/Ports.java`
- Test: `rules/PortsTest.java`

**Interfaces:**
- Produces:
```java
public final class Ports {
  public record CanId(int id, String bus) {}
  public record Device(String name, CanId can, String kind) {}   // kind: "TalonFX", "CANcoder", "Pigeon2", "CANdi", "CANrange", "PDH"
  public static final String DRIVE_BUS = "drive", RIO_BUS = "rio";
  public static final CanId PDH, PIGEON,
      FL_DRIVE, FL_STEER, FL_ENCODER, FR_DRIVE, FR_STEER, FR_ENCODER,
      BL_DRIVE, BL_STEER, BL_ENCODER, BR_DRIVE, BR_STEER, BR_ENCODER,
      ELEVATOR_LEFT, ELEVATOR_RIGHT, WRIST, WRIST_CANDI,
      CORAL_ROLLERS, CLAW_FRONT, CLAW_REAR, CORAL_RANGE, ALGAE_RANGE;
  public static final int ELEVATOR_HOME_DIO = 0;
  public static List<Device> all();
}
```
- Values: PDH 1 rio; PIGEON 2 drive; FL 11/12/13, FR 21/22/23, BL 31/32/33, BR 41/42/43 drive (drive, steer, encoder); ELEVATOR_LEFT 50, ELEVATOR_RIGHT 51, WRIST 52, WRIST_CANDI 53, CORAL_ROLLERS 55, CLAW_FRONT 56, CLAW_REAR 57, CORAL_RANGE 58, ALGAE_RANGE 59, all rio. IDs 60 to 62 stay unused.

- [ ] **Step 1: Write the failing test**
```java
@Test void idsAreUniqueAcrossBothBuses() {
  var ids = Ports.all().stream().map(d -> d.can().id()).toList();
  assertEquals(ids.size(), new HashSet<>(ids).size());
}
@Test void idsFitCtreRange() { Ports.all().forEach(d -> assertTrue(d.can().id() >= 0 && d.can().id() <= 62)); }
@Test void deviceCounts() {
  assertEquals(23, Ports.all().size());
  assertEquals(13, Ports.all().stream().filter(d -> d.can().bus().equals(Ports.DRIVE_BUS)).count());
  assertEquals(10, Ports.all().stream().filter(d -> d.can().bus().equals(Ports.RIO_BUS)).count());
}
@Test void reservedIdsAreFree() { Ports.all().forEach(d -> assertFalse(d.can().id() >= 60)); }
```
- [ ] **Step 2:** Run `./gradlew test --tests '*PortsTest'`. Expected: FAIL, `Ports` not found.
- [ ] **Step 3:** Implement `Ports` with the values above.
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Commit `feat: CAN map`.

### Task 3: Constants and setpoints

**Files:**
- Create: `config/RobotConstants.java`, `config/Setpoints.java`
- Test: `rules/UnitsTest.java`

**Interfaces:**
- Produces `RobotConstants` (nested static classes, all `public static final double` unless noted):
  - top level: `boolean PRO_LICENSED = true`, `int TEAM_NUMBER = 4206`, `Mode currentMode()`.
  - `DriveK`: `MODULE_OFFSET_IN = 11.373`, `WHEEL_DIAMETER_IN = 3.947`, `DRIVE_RATIO = 6.75`, `STEER_RATIO = 150.0 / 7.0`, `BUMPER_FACE_FROM_CENTRE_IN = 17.4` (PLACEHOLDER drawing), `MAX_SPEED_MPS = 4.67`, `ROBOT_MASS_LB = 140` (PLACEHOLDER drawing).
  - `ElevatorK`: `SPROCKET_TEETH = 22`, `CHAIN_PITCH_IN = 0.25`, `GEAR_RATIO = 12`, `STAGES = 3`, `IN_PER_MOTOR_ROT` (computed, 1.375), `TRAVEL_IN = 66`, `SOFT_MAX_IN = 65.25`, `MAX_ACCEL_IN_PER_S2 = 0.6 * 386.09`, `CRUISE_IN_PER_S = 110` (PLACEHOLDER tuning), `PIVOT_REST_HEIGHT_IN = 24.5`, `PIVOT_X_IN = 4.5`, `CARRIAGE_GROUP_LB = 25`, `STAGE_A_LB = 5` and `STAGE_B_LB = 4` (PLACEHOLDER drawing), `HOME_VOLTS = -1.5`, `HOME_TIMEOUT_S = 3.0`.
  - `WristK`: `GEAR_RATIO = 60`, `MIN_DEG = -90`, `MAX_DEG = 75`, `HARD_MIN_DEG = -93`, `HARD_MAX_DEG = 77`, `STATOR_LIMIT_A = 40`, `TURNING_LB = 15.3`, `GRAVITY_LBIN_EMPTY = 125`, `GRAVITY_LBIN_CORAL = 138`, `GRAVITY_LBIN_ALGAE = 151`, `MOI_KGM2 = 0.39` (PLACEHOLDER drawing), `CRUISE_DEG_PER_S = 400` and `ACCEL_DEG_PER_S2 = 1600` (PLACEHOLDER tuning).
  - `CoralK`: `GEAR_RATIO = 3`, `WHEEL_DIAMETER_IN = 2`. `ClawK`: `GEAR_RATIO = 2`, `STATOR_LIMIT_A = 30`.
  - `CameraK`: `HEIGHT_IN = 9.57`, `PITCH_UP_DEG = 12`, `FORWARD_IN = 12.0` (PLACEHOLDER drawing), `H_FOV_DEG = 82`, `V_FOV_DEG = 56.2`, `H_PIXELS = 1280`, `V_PIXELS = 800`.
- Produces `Setpoints`:
```java
public enum Pose {
  STOW(0, -90), STATION(1.75, -30), L1(5, -35), L2(17.5, -35), L3(33.3, -35), L4(65.25, -52),
  ALGAE_LOW(8.8, 46), ALGAE_HIGH(24.6, 46), ALGAE_CARRY(1.75, 45), NET(65.25, 75), PROCESSOR(1.75, 15);
  public final double carriageIn, wristDeg;
}
public static final double CARRIAGE_TOL_IN = 0.5, WRIST_TOL_DEG = 1.5;
```
  STATION, L1 and PROCESSOR carry `PLACEHOLDER(drawing)`; STATION and L2 also carry `PLACEHOLDER(funnel-redesign)`.

- [ ] **Step 1: Write the failing test**
```java
@Test void oneMotorTurnIsOnePoint375Inches() { assertEquals(1.375, ElevatorK.IN_PER_MOTOR_ROT, 1e-9); }
@Test void fullTravelIs48MotorTurns() { assertEquals(48.0, ElevatorK.TRAVEL_IN / ElevatorK.IN_PER_MOTOR_ROT, 1e-9); }
@Test void accelCapIsPointSixG() { assertEquals(231.65, ElevatorK.MAX_ACCEL_IN_PER_S2, 0.01); }
@Test void setpointsAreInsideSoftLimits() {
  for (var p : Setpoints.Pose.values()) {
    assertTrue(p.carriageIn >= 0 && p.carriageIn <= ElevatorK.SOFT_MAX_IN, p.name());
    assertTrue(p.wristDeg >= WristK.MIN_DEG && p.wristDeg <= WristK.MAX_DEG, p.name());
  }
}
```
- [ ] **Step 2:** Run `./gradlew test --tests '*UnitsTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement both files.
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Commit `feat: constants and setpoints`.

### Task 4: Keep-out table

**Files:**
- Create: `config/KeepOut.java`
- Test: `rules/KeepOutTest.java`

**Interfaces:**
- Produces:
```java
public final class KeepOut {
  public record Zone(double minCarriageIn, double maxCarriageIn, double minWristDeg, double maxWristDeg) {}
  public record Range(double min, double max) {}
  public static final double STOW_TOL_DEG = 2.0, ALGAE_MIN_WRIST_DEG = 10.0;
  public static final List<Zone> ZONES;   // [0,1.25): -90..-90   [1.25,6): -90..75   [6,23): -38..75   [23,65.25]: -90..75
  public static boolean isLegal(double carriageIn, double wristDeg);
  public static int zoneIndexAt(double carriageIn);
  public static Range wristRangeAt(double carriageIn);
  public static Range carriageRangeAt(double carriageIn, double wristDeg);
}
```
- A zone covers `min <= carriage < max`; the last zone includes its top. In the stow-only zone a wrist within `STOW_TOL_DEG` of −90 is legal. The `-38` carries `PLACEHOLDER(funnel-redesign)` with the note "Rev A gave −35".
- `carriageRangeAt` returns the largest connected span of carriage heights that contains `carriageIn` and in which `wristDeg` is legal.

- [ ] **Step 1: Write the failing test**
```java
@Test void stowOnlyAtTheBottom() { assertTrue(KeepOut.isLegal(0, -90)); assertFalse(KeepOut.isLegal(0.5, -60)); }
@Test void anyAngleBetween1_25And6() { assertTrue(KeepOut.isLegal(3, -90)); assertTrue(KeepOut.isLegal(3, 75)); }
@Test void funnelZoneLimit() { assertFalse(KeepOut.isLegal(10, -40)); assertTrue(KeepOut.isLegal(10, -38)); }
@Test void anyAngleAbove23() { assertTrue(KeepOut.isLegal(40, -90)); }
@Test void everySetpointIsLegal() { for (var p : Setpoints.Pose.values()) assertTrue(KeepOut.isLegal(p.carriageIn, p.wristDeg), p.name()); }
@Test void stowedWristCanReachOnlyUpTo6() { var r = KeepOut.carriageRangeAt(0, -90); assertEquals(0, r.min(), 1e-9); assertEquals(6, r.max(), 1e-9); }
@Test void levelWristCannotGoBelow1_25() { var r = KeepOut.carriageRangeAt(30, 0); assertEquals(1.25, r.min(), 1e-9); assertEquals(65.25, r.max(), 1e-9); }
@Test void steepWristAboveFunnelStaysAbove23() { assertEquals(23, KeepOut.carriageRangeAt(40, -60).min(), 1e-9); }
```
- [ ] **Step 2:** Run `./gradlew test --tests '*KeepOutTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement `KeepOut`.
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Commit `feat: keep-out table`.

### Task 5: Superstructure planner

**Files:**
- Create: `subsystems/SuperstructurePlanner.java`
- Test: `rules/SuperstructurePlannerTest.java`

**Interfaces:**
- Consumes: `KeepOut`, `Setpoints.Pose`.
- Produces:
```java
public final class SuperstructurePlanner {
  public record Targets(double carriageIn, double wristDeg) {}
  public static final double MARGIN_IN = 0.5, MARGIN_DEG = 3.0;
  public static Targets next(double carriageIn, double wristDeg, Targets goal, boolean hasAlgae);
}
```

- [ ] **Step 1: Write the failing test.** The sweep steps a kinematic model: each 10 ms tick moves the carriage at most 1.1 in and the wrist at most 4° toward the planner's targets.
```java
@Test void everyMoveStaysLegalAndFinishes() {
  for (var from : Setpoints.Pose.values()) for (var to : Setpoints.Pose.values()) {
    double h = from.carriageIn, w = from.wristDeg;
    var goal = new Targets(to.carriageIn, to.wristDeg);
    int ticks = 0;
    while (Math.abs(h - goal.carriageIn()) > 0.01 || Math.abs(w - goal.wristDeg()) > 0.01) {
      var t = SuperstructurePlanner.next(h, w, goal, false);
      h += MathUtil.clamp(t.carriageIn() - h, -1.1, 1.1);
      w += MathUtil.clamp(t.wristDeg() - w, -4, 4);
      assertTrue(KeepOut.isLegal(h, w), from + " to " + to + " at h=" + h + " w=" + w);
      assertTrue(++ticks < 2000, from + " to " + to + " did not finish");
    }
  }
}
@Test void stowToL2WaitsBelowSixUntilWristIsUp() {
  var t = SuperstructurePlanner.next(0, -90, new Targets(17.5, -35), false);
  assertEquals(5.5, t.carriageIn(), 1e-9); assertEquals(-90, t.wristDeg(), 1e-9);
}
@Test void stowToL4TurnsTheWristThroughTheFunnelZoneFirst() {          // L4's -52 is illegal between 6 and 23 in
  var t = SuperstructurePlanner.next(5.5, -90, new Targets(65.25, -52), false);
  assertEquals(-35, t.wristDeg(), 1e-9); assertEquals(5.5, t.carriageIn(), 1e-9);
}
@Test void algaeNeverDropsBelowTen() {
  var t = SuperstructurePlanner.next(8.8, 46, new Targets(0, -90), true);
  assertTrue(t.wristDeg() >= 10); assertTrue(t.carriageIn() >= 1.75);
}
```
- [ ] **Step 2:** Run `./gradlew test --tests '*SuperstructurePlannerTest'`. Expected: FAIL.
- [ ] **Step 3: Implement `next`** with this algorithm, which the signature does not determine. It was checked with a throwaway sweep of all 121 moves at four step sizes before this plan was issued.
  1. If `hasAlgae`, raise the goal wrist angle to at least `ALGAE_MIN_WRIST_DEG`.
  2. A zone's *margined* wrist range is its range with the lower bound raised by `MARGIN_DEG` when that bound is above `WristK.MIN_DEG`. The stow zone's single point stays −90.
  3. Wrist target: start from the margined range of the zone the carriage is in. If the goal height is in a different zone, intersect it with the margined range of the next zone toward the goal, one zone only, and keep the intersection when it is not empty. Clamp the goal wrist angle into the result. If `hasAlgae`, raise the target to at least `ALGAE_MIN_WRIST_DEG` again.
  4. Carriage target: take `KeepOut.carriageRangeAt(carriageIn, wristDeg)`. Pull in by `MARGIN_IN` each end that is not a soft limit (0 or 65.25), then clamp the goal height into it.
  5. Return both targets.
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Commit `feat: superstructure planner`.

### Task 6: Field geometry

**Files:**
- Create: `field/AllianceFlip.java`, `field/Reef.java`, `field/Station.java`, `field/StartPose.java`
- Test: `rules/ReefTest.java`, `rules/StationTest.java`

**Interfaces:**
- Produces:
```java
public final class AllianceFlip { public static Pose2d rotate(Pose2d blue); public static Translation2d rotate(Translation2d blue); }
public final class Reef {
  public enum Branch { A, B, C, D, E, F, G, H, I, J, K, L; public int face(); public boolean isLeft(); }
  public static final double BRANCH_OFFSET_IN = 6.47, ALGAE_STANDOFF_IN = 0.5;
  public static Pose2d tagPose(Alliance a, int face);
  public static Pose2d scoringPose(Alliance a, Branch b);
  public static Pose2d algaePose(Alliance a, int face);
  public static boolean algaeIsHigh(int face);                    // faces 0 (AB), 2 (EF), 4 (IJ)
  public static int nearestFace(Alliance a, Translation2d robot);
  public static boolean pathClear(Alliance a, Translation2d from, Translation2d to);
  public static List<Pose2d> route(Alliance a, Translation2d from, Pose2d to);   // via points, then `to`
}
public final class Station { public enum Side { LEFT, RIGHT } public static Pose2d pickupPose(Alliance a, Side s); public static Side nearest(Alliance a, Translation2d robot); }
public enum StartPose { LEFT, CENTRE, RIGHT; public Pose2d pose(Alliance a); }
```
- Tags come from `AprilTagFields.k2025ReefscapeWelded`. Face order AB, CD, EF, GH, IJ, KL maps to blue tags 18, 17, 22, 21, 20, 19 and red tags 7, 8, 9, 10, 11, 6. Stations: blue right 12, blue left 13, red right 2, red left 1.
- `scoringPose`: robot faces the tag, centre `BUMPER_FACE_FROM_CENTRE_IN` out along the tag normal, shifted `BRANCH_OFFSET_IN` to the robot's left for a left branch. `algaePose`: centred on the tag, `ALGAE_STANDOFF_IN` further out. `pickupPose`: heading equals the tag's, so the rear faces the wall, centre `BUMPER_FACE_FROM_CENTRE_IN` out from the tag.
- `pathClear` is false when the segment passes within 1.69 m of the reef centre. `route` inserts via points on a 1.75 m circle at 30° + 60°·k around the reef centre, taking the shorter way round, each with the heading of `to`.
- `StartPose` blue values: x 7.2 m, heading 180°, y 2.0 (RIGHT), 4.026 (CENTRE), 6.05 (LEFT), all `PLACEHOLDER(field-check)`.

- [ ] **Step 1: Write the failing tests**
```java
@Test void branchPosesSitSquareAtBumperDistance() {
  for (var a : Alliance.values()) for (var b : Reef.Branch.values()) {
    var tag = Reef.tagPose(a, b.face()); var robot = Reef.scoringPose(a, b);
    var rel = robot.relativeTo(tag);                       // tag frame: x out of the face
    assertEquals(Units.inchesToMeters(17.4), rel.getX(), 1e-6);
    assertEquals(Units.inchesToMeters(6.47), Math.abs(rel.getY()), 1e-6);
    assertEquals(180, Math.abs(rel.getRotation().getDegrees()), 1e-6);
  }
}
@Test void leftBranchIsOnTheRobotsLeft() {
  var left = Reef.scoringPose(Alliance.Blue, Reef.Branch.A); var right = Reef.scoringPose(Alliance.Blue, Reef.Branch.B);
  assertTrue(right.relativeTo(left).getY() < 0);
}
@Test void redIsBlueRotated() { for (var b : Reef.Branch.values()) assertPoseEquals(AllianceFlip.rotate(Reef.scoringPose(Alliance.Blue, b)), Reef.scoringPose(Alliance.Red, b), 1e-3); }
@Test void faceAbFacesTheAllianceWall() { assertEquals(180, Math.abs(Reef.tagPose(Alliance.Blue, 0).getRotation().getDegrees()), 1e-6); }
@Test void routesClearTheReef() {
  var from = Reef.scoringPose(Alliance.Blue, Reef.Branch.E).getTranslation();
  var to = Station.pickupPose(Alliance.Blue, Station.Side.LEFT);
  var pts = Reef.route(Alliance.Blue, from, to); var prev = from;
  for (var p : pts) { assertTrue(Reef.pathClear(Alliance.Blue, prev, p.getTranslation())); prev = p.getTranslation(); }
  assertEquals(to, pts.get(pts.size() - 1));
}
@Test void stationRearFacesWall() { /* pickupPose heading equals tag heading; red right is blue right rotated */ }
```
- [ ] **Step 2:** Run `./gradlew test --tests '*ReefTest' --tests '*StationTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement the four files.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: field geometry`.

### Task 7: Single-tag position fix

**Files:**
- Create: `subsystems/vision/CameraModel.java`, `subsystems/vision/SingleTagFix.java`
- Test: `rules/SingleTagFixTest.java`

**Interfaces:**
- Produces:
```java
public record TagObservation(int tagId, double txDeg, double heightPx) {}      // tx positive to the right, as Limelight reports
public record CameraModel(Transform3d robotToCamera, double hFovDeg, double vFovDeg, int hPixels, int vPixels) {
  public static CameraModel limelight4();                                       // from RobotConstants.CameraK
  public Optional<TagObservation> project(Pose2d robot, int tagId, Pose3d tag, double noisePx, Random rng);
}
public final class SingleTagFix {
  public static final double TAG_SIZE_M = Units.inchesToMeters(6.5);
  public static double distanceM(double heightPx, CameraModel cam, double tagBottomHeightM);
  public static Translation2d robotPosition(TagObservation obs, CameraModel cam, Rotation2d gyro, Pose3d tag);
}
```
- `project` returns empty when the tag is more than 4 m away, outside half the horizontal field of view, or facing away.

- [ ] **Step 1: Write the failing test**
```java
@ParameterizedTest @ValueSource(doubles = {0.6, 1.0, 2.0, 3.0})
void recoversAKnownPose(double metresOut) {
  var tag = layout.getTagPose(18).get(); var cam = CameraModel.limelight4();
  var truth = Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(metresOut, 0.2, Rotation2d.k180deg));
  var obs = cam.project(truth, 18, tag, 0, new Random(1)).orElseThrow();
  var fix = SingleTagFix.robotPosition(obs, cam, truth.getRotation(), tag);
  assertEquals(0, fix.getDistance(truth.getTranslation()), metresOut <= 1.0 ? 0.03 : 0.08);
}
@Test void tagBehindTheCameraIsNotSeen() { /* robot facing away: project(...) is empty */ }
```
- [ ] **Step 2:** Run `./gradlew test --tests '*SingleTagFixTest'`. Expected: FAIL.
- [ ] **Step 3: Implement.** The method is 2910's, written fresh:
  - `project`: bearing from the camera to the tag centre gives `txDeg`; `heightPx` is the difference between the elevation angles to the tag's top and bottom edges, times `vPixels / vFovDeg`.
  - `distanceM`: with θ = `heightPx · vFovDeg / vPixels`, T = tag size and h = tag bottom height minus camera height, solve tan θ = T·d / (d² + h·(h + T)) for the larger root: d = (T + √(T² − 4·tan²θ·h·(h + T))) / (2·tan θ).
  - `robotPosition`: camera = tag − d at angle (gyro + camera yaw − tx); robot = camera − (robot-to-camera translation rotated by gyro).
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Commit `feat: single-tag position fix`.

### Task 8: Drive-to-point controller

**Files:**
- Create: `subsystems/drive/DriveToPoint.java`
- Test: `rules/DriveToPointTest.java`

**Interfaces:**
- Produces:
```java
public final class DriveToPoint {
  public record Output(double vxMps, double vyMps) {}
  public static final double KP_AUTO = 3.0, KP_TELEOP = 3.6, KD = 0.1, FRICTION_FRACTION = 0.02, DONE_M = 0.01;
  public DriveToPoint(double kP, double kD);
  public Output calculate(Translation2d current, Translation2d target, double maxSpeedMps, double dtSeconds);
}
```
- Speed = min(|PD on straight-line distance| + `FRICTION_FRACTION · DriveK.MAX_SPEED_MPS`, `maxSpeedMps`), pointed at the target; zero inside `DONE_M`.

- [ ] **Step 1: Write the failing test**
```java
@Test void pointsAtTheTarget() { var o = c.calculate(new Translation2d(), new Translation2d(1, 1), 3, 0.02); assertEquals(o.vxMps(), o.vyMps(), 1e-9); assertTrue(o.vxMps() > 0); }
@Test void capsAtMaxSpeed() { var o = c.calculate(new Translation2d(), new Translation2d(5, 0), 2.0, 0.02); assertEquals(2.0, Math.hypot(o.vxMps(), o.vyMps()), 1e-9); }
@Test void stopsInsideTolerance() { var o = c.calculate(new Translation2d(), new Translation2d(0.005, 0), 2.0, 0.02); assertEquals(0, o.vxMps(), 1e-12); }
```
- [ ] **Step 2:** Run `./gradlew test --tests '*DriveToPointTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Commit `feat: drive-to-point controller`.

---

# Milestone 2: mechanisms against physics

Ends with `./gradlew simTest --tests 'frc.robot.mechanism.*'` passing.

### Task 9: Elevator

**Files:**
- Create: `subsystems/elevator/{ElevatorIO,ElevatorIOTalonFX,ElevatorIOSim,Elevator}.java`, `testutil/SimHarness.java`
- Test: `mechanism/ElevatorSimTest.java`

**Interfaces:**
- Produces:
```java
public interface ElevatorIO {
  @AutoLog class ElevatorIOInputs { public boolean connected, followerConnected, homeSwitch; public double carriageIn, velocityInPerS, appliedVolts, statorAmps, supplyAmps; }
  default void updateInputs(ElevatorIOInputs in) {}
  default void setTargetIn(double carriageIn) {}
  default void setVolts(double volts) {}
  default void zero() {}
  default void stop() {}
}
public class ElevatorIOTalonFX implements ElevatorIO { public ElevatorIOTalonFX(CanId left, CanId right, int homeDio); }
public class ElevatorIOSim extends ElevatorIOTalonFX { public void simulate(double dtSeconds); public void setCarriageIn(double in); public void setHomeSwitchBroken(boolean broken); }
public class Elevator extends SubsystemBase {
  public Elevator(ElevatorIO io);
  public void setGoalIn(double carriageIn); public double carriageIn(); public double velocityInPerS();
  public boolean atGoal(); public boolean isHomed(); public void requestHome();
}
public final class SimHarness {                      // steps the HAL, Phoenix and the scheduler in 20 ms real-time ticks
  public static void enable(); public static void disable(); public static void run(double seconds); public static void register(Runnable simStep);
}
```
- `ElevatorIOTalonFX` config: `SensorToMechanismRatio = 1 / IN_PER_MOTOR_ROT` so Phoenix positions read in inches; Motion Magic cruise `CRUISE_IN_PER_S`, acceleration `MAX_ACCEL_IN_PER_S2`; gravity type elevator-static; soft limits 0 and `SOFT_MAX_IN`; the right motor follows the left; FOC when `PRO_LICENSED`.
- `ElevatorIOSim` drives a WPILib `ElevatorSim` with two Kraken X60 (FOC), gearing 12, drum radius 16.5 in / 2π, mass (STAGE_A + 2·STAGE_B + 3·CARRIAGE_GROUP) / 3 lb, range 0 to 66 in, gravity on. Each `simulate` reads the simulated motor voltage, updates the model, and writes rotor position and velocity back. The home DIO reads true at or below 0.05 in unless broken.
- `Elevator`: un-homed at boot. On first enable it applies `HOME_VOLTS` until the switch closes, then zeroes. If the switch has not closed after `HOME_TIMEOUT_S` it zeroes once velocity has been under 0.5 in/s for 0.25 s and raises alert "Elevator home switch not seen". Goals are ignored until homed. `atGoal` uses `Setpoints.CARRIAGE_TOL_IN`.

- [ ] **Step 1: Write the failing tests** (all `@Tag("sim")`)
```java
@Test void homesFromAnUnknownHeight() { sim.setCarriageIn(7.0); SimHarness.enable(); SimHarness.run(4); assertTrue(elevator.isHomed()); assertEquals(0, elevator.carriageIn(), 0.3); }
@ParameterizedTest @EnumSource(Setpoints.Pose.class)
void reachesEverySetpoint(Setpoints.Pose p) { homeThen(() -> elevator.setGoalIn(p.carriageIn)); SimHarness.run(2.0); assertEquals(p.carriageIn, elevator.carriageIn(), 0.5); }
@Test void neverPassesTheSoftLimit() { homeThen(() -> elevator.setGoalIn(80)); runRecording(2.5); assertTrue(maxHeight <= 65.25 + 0.3); }
@Test void downwardAccelerationStaysUnderPointSixG() {
  homeThen(() -> elevator.setGoalIn(65.25)); runRecording(2.0); elevator.setGoalIn(0); runRecording(2.0);
  assertTrue(minAccelInPerS2 >= -231.65 * 1.15);          // 15 % allowance for differentiation noise
}
@Test void fullTravelTakesUnderOnePointFiveSeconds() { /* 0 to 65.25: atGoal within 1.5 s */ }
@Test void brokenHomeSwitchStillHomesOnStall() { sim.setHomeSwitchBroken(true); sim.setCarriageIn(5); SimHarness.enable(); SimHarness.run(5); assertTrue(elevator.isHomed()); assertTrue(alertActive("Elevator home switch not seen")); }
@Test void ignoresGoalsUntilHomed() { elevator.setGoalIn(30); SimHarness.run(0.2); assertTrue(elevator.carriageIn() < 8); }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*ElevatorSimTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement `SimHarness`, then the four elevator files. Tune `kG`, `kV`, `kP`, `kD` in simulation until the tests pass; mark each gain `PLACEHOLDER(tuning)`.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: elevator with physics simulation`.

### Task 10: Wrist

**Files:**
- Create: `subsystems/wrist/{WristIO,WristIOTalonFX,WristIOSim,Wrist}.java`
- Test: `mechanism/WristSimTest.java`

**Interfaces:**
- Consumes: `SimHarness`.
- Produces:
```java
public interface WristIO {
  @AutoLog class WristIOInputs { public boolean connected, encoderConnected; public double wristDeg, absoluteDeg, velocityDegPerS, appliedVolts, statorAmps; }
  default void updateInputs(WristIOInputs in) {}
  default void setTargetDeg(double deg) {}
  default void stop() {}
}
public class WristIOTalonFX implements WristIO { public WristIOTalonFX(CanId motor, CanId candi); }
public class WristIOSim extends WristIOTalonFX { public void simulate(double dtSeconds); public void setWristDeg(double deg); public void setGravityTorqueLbIn(double lbIn); public void setEncoderConnected(boolean connected); }
public class Wrist extends SubsystemBase {
  public Wrist(WristIO io);
  public void setGoalDeg(double deg); public double wristDeg(); public boolean atGoal(); public boolean isTrusted();
}
```
- `WristIOTalonFX` config: feedback from the CANdi's PWM input 1, fused when `PRO_LICENSED` and remote otherwise; `RotorToSensorRatio = 60`; gravity type arm-cosine with zero at level; soft limits `MIN_DEG` and `MAX_DEG`; stator limit `STATOR_LIMIT_A`; Motion Magic from `CRUISE_DEG_PER_S` and `ACCEL_DEG_PER_S2`.
- `WristIOSim` uses WPILib `SingleJointedArmSim` with one Kraken X60 (FOC), gearing 60, `MOI_KGM2`, limits `HARD_MIN_DEG` and `HARD_MAX_DEG`, starting at −90. That model applies gravity as 3g/(2L)·cos θ, so the arm length passed in is L = 3·g·MOI / (2·τ) with τ the gravity torque in N·m (0.4065 m for 125 lb-in). If Phoenix does not simulate the CANdi, feed `absoluteDeg` at the IO layer instead and record that in `docs/open-numbers.md`.
- `Wrist.isTrusted()` is false when the encoder is disconnected or `absoluteDeg` is outside −95 to +80. An untrusted wrist never commands motion and raises alert "Wrist encoder not trusted".

- [ ] **Step 1: Write the failing tests** (all `@Tag("sim")`)
```java
@ParameterizedTest @EnumSource(Setpoints.Pose.class)
void reachesEverySetpoint(Setpoints.Pose p) { wrist.setGoalDeg(p.wristDeg); SimHarness.run(1.5); assertEquals(p.wristDeg, wrist.wristDeg(), 1.5); }
@Test void staysUnderFortyAmps() { wrist.setGoalDeg(75); runRecording(1.5); assertTrue(maxStatorAmps <= 40 + 2); }
@Test void holdsAnAlgaeAtLevel() { sim.setGravityTorqueLbIn(151); wrist.setGoalDeg(0); SimHarness.run(2); assertEquals(0, wrist.wristDeg(), 1.5); }
@Test void doesNotMoveWithoutItsEncoder() { sim.setEncoderConnected(false); wrist.setGoalDeg(0); SimHarness.run(1); assertEquals(-90, wrist.wristDeg(), 2); assertFalse(wrist.isTrusted()); assertTrue(alertActive("Wrist encoder not trusted")); }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*WristSimTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement. Tune gains in simulation; mark them `PLACEHOLDER(tuning)`.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: wrist with physics simulation`.

### Task 11: Coral rollers and algae claw

**Files:**
- Create: `subsystems/coral/{CoralRollersIO,CoralRollersIOTalonFX,CoralRollersIOSim,CoralRollers}.java`, `subsystems/claw/{AlgaeClawIO,AlgaeClawIOTalonFX,AlgaeClawIOSim,AlgaeClaw}.java`
- Test: `mechanism/RollersSimTest.java`

**Interfaces:**
- Produces (the claw mirrors the coral classes with `Algae` names):
```java
public interface CoralRollersIO {
  @AutoLog class CoralRollersIOInputs { public boolean connected, rangeConnected; public double velocityRps, appliedVolts, statorAmps, rangeM; }
  default void updateInputs(CoralRollersIOInputs in) {}
  default void setVolts(double volts) {}             // positive pulls the piece in
}
public class CoralRollersIOSim extends CoralRollersIOTalonFX { public void simulate(double dt); public void setCoralPresent(boolean present); public void setRangeConnected(boolean connected); }
public class CoralRollers extends SubsystemBase { public void intake(); public void hold(); public void eject(); public void stop(); public boolean hasCoral(); public boolean isEjecting(); }
public class AlgaeClaw extends SubsystemBase { public void intake(); public void hold(); public void eject(); public void stop(); public boolean hasAlgae(); public boolean isEjecting(); }
```
- Voltages: coral intake 4, hold 0.5, eject −6; claw intake 6, hold 1.5, eject −8, all `PLACEHOLDER(tuning)`. `intake()` switches itself to `hold()` when the piece is seen.
- A piece is present when `rangeM < 0.05` for 60 ms, and absent when above it for 60 ms. A disconnected CANrange reads absent and raises alert "Coral sensor offline" or "Algae sensor offline".
- The claw's two motors are inverted against each other in config, so one voltage runs both inward. Stator limit `ClawK.STATOR_LIMIT_A`.

- [ ] **Step 1: Write the failing tests** (all `@Tag("sim")`)
```java
@Test void intakeStopsWhenCoralIsSeen() { coral.intake(); SimHarness.run(0.2); sim.setCoralPresent(true); SimHarness.run(0.2); assertTrue(coral.hasCoral()); assertEquals(0.5, lastVolts(), 0.2); }
@Test void aBounceDoesNotClearTheCoral() { seat(); sim.setCoralPresent(false); SimHarness.run(0.03); sim.setCoralPresent(true); SimHarness.run(0.1); assertTrue(coral.hasCoral()); }
@Test void offlineSensorReadsEmptyAndAlerts() { seat(); sim.setRangeConnected(false); SimHarness.run(0.2); assertFalse(coral.hasCoral()); assertTrue(alertActive("Coral sensor offline")); }
@Test void clawStaysUnderThirtyAmps() { claw.intake(); runRecording(1); assertTrue(maxStatorAmps <= 30 + 2); }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*RollersSimTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement both mechanisms. If Phoenix does not simulate the CANrange, feed `rangeM` at the IO layer and record that in `docs/open-numbers.md`.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: coral rollers and algae claw`.

### Task 12: Superstructure

**Files:**
- Create: `subsystems/Superstructure.java`
- Test: `mechanism/SuperstructureSimTest.java`

**Interfaces:**
- Consumes: `Elevator`, `Wrist`, `CoralRollers`, `AlgaeClaw`, `SuperstructurePlanner`, `Setpoints.Pose`.
- Produces:
```java
public class Superstructure extends SubsystemBase {
  public enum Wanted { STOW, STATION_INTAKE, L1, L2, L3, L4, ALGAE_LOW, ALGAE_HIGH, NET, PROCESSOR }
  public Superstructure(Elevator e, Wrist w, CoralRollers c, AlgaeClaw a);
  public void setWanted(Wanted w); public Wanted wanted();
  public void requestRelease(boolean release);
  public boolean atPose(); public boolean hasCoral(); public boolean hasAlgae();
  public double driveSpeedScale();
  public Command setWantedCommand(Wanted w);
}
```
- Each loop: map `wanted` to a `Setpoints.Pose`, call `SuperstructurePlanner.next` with measured positions, and send the result to the elevator and wrist. `STOW` with an algae on board uses `ALGAE_CARRY`.
- Rollers: `STATION_INTAKE` runs coral intake; `ALGAE_LOW` and `ALGAE_HIGH` run claw intake; a release request ejects the matching piece only while `atPose()` is true. `setWanted` clears a pending release.
- `driveSpeedScale()` is 1.0 up to 23 in of carriage and falls linearly to 0.35 at 65.25 in (`PLACEHOLDER(tuning)`).
- If the wrist is not trusted, the elevator goal is held at or below 1.0 in.
- Logs `Superstructure/Wanted`, `Superstructure/AtPose`, `Superstructure/TargetCarriageIn`, `Superstructure/TargetWristDeg`.

- [ ] **Step 1: Write the failing tests** (all `@Tag("sim")`)
```java
@Test void everyTransitionStaysLegalInPhysics() {
  for (var from : Wanted.values()) for (var to : Wanted.values()) {
    goTo(from); s.setWanted(to);
    runChecking(3.0, () -> assertTrue(KeepOut.isLegal(elevator.carriageIn(), wrist.wristDeg()), from + " to " + to));
    assertTrue(s.atPose(), from + " to " + to);
  }
}
@Test void releaseWaitsForThePose() { seatCoral(); s.setWanted(Wanted.L4); s.requestRelease(true); SimHarness.run(0.2); assertFalse(coral.isEjecting()); SimHarness.run(2.5); assertTrue(coral.isEjecting()); }
@Test void stowWithAlgaeUsesCarry() { giveAlgae(); s.setWanted(Wanted.STOW); SimHarness.run(2.5); assertTrue(wrist.wristDeg() >= 10); assertTrue(elevator.carriageIn() >= 1.25); }
@Test void cancelMidScoreReturnsLegallyAndKeepsTheSpeedCap() {
  seatCoral(); s.setWanted(Wanted.L4); SimHarness.run(0.6); s.setWanted(Wanted.STOW);
  runChecking(3.0, () -> { assertTrue(KeepOut.isLegal(elevator.carriageIn(), wrist.wristDeg())); if (elevator.carriageIn() > 23.5) assertTrue(s.driveSpeedScale() < 1.0); });
  assertTrue(s.atPose());
}
@Test void untrustedWristKeepsTheElevatorDown() { wristSim.setEncoderConnected(false); s.setWanted(Wanted.L3); SimHarness.run(2); assertTrue(elevator.carriageIn() <= 1.25); }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*SuperstructureSimTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement `Superstructure`.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: superstructure`.

---

# Milestone 3: drivetrain, vision and teleop

Ends with a robot that can be driven in `simulateJava` and a scripted teleop cycle test passing.

### Task 13: Drivetrain

**Files:**
- Create: `subsystems/drive/{TunerConstants,SwerveIO,SwerveIOCTRE,SwerveIOSim,SwerveSubsystem}.java`, `RobotState.java`
- Test: `match/DriveSimTest.java`

**Interfaces:**
- Consumes: `DriveToPoint`, `Ports`, `RobotConstants.DriveK`.
- Produces:
```java
public interface SwerveIO {
  @AutoLog class SwerveIOInputs { public Pose2d pose = new Pose2d(); public ChassisSpeeds speeds = new ChassisSpeeds(); public SwerveModuleState[] moduleStates, moduleTargets; public Rotation2d gyroYaw = new Rotation2d(); public boolean connected; }
  default void updateInputs(SwerveIOInputs in) {}
  default void driveFieldRelative(double vxMps, double vyMps, double omegaRadPerS) {}
  default void driveFieldRelativeFacing(double vxMps, double vyMps, Rotation2d heading, double maxOmegaRadPerS) {}
  default void resetPose(Pose2d pose) {}
  default void resetTranslation(Translation2d t) {}
  default void stop() {}
}
public class SwerveIOSim extends SwerveIOCTRE { public void simulate(double dt); public Pose2d truePose(); public void setTruePose(Pose2d p); }
public class SwerveSubsystem extends SubsystemBase {
  public enum Wanted { IDLE, TELEOP, DRIVE_TO_POINT }
  public void setTeleopInput(double x, double y, double omega);            // each in -1..1, already alliance-corrected
  public void driveToPoint(Pose2d target, double maxSpeedMps, boolean teleopGains);
  public boolean atTarget(); public double distanceToTargetM();
  public void setSpeedScale(double scale);
  public void resetPose(Pose2d pose); public void resetTranslation(Translation2d t); public Pose2d pose();
}
public final class RobotState { public static RobotState get(); public Pose2d pose(); public ChassisSpeeds speeds(); public Optional<Alliance> alliance(); public Superstructure.Wanted selectedLevel(); public void selectLevel(Superstructure.Wanted l); }
```
- `TunerConstants` is hand-written in the shape CTRE's generator produces: module positions (±`MODULE_OFFSET_IN`, ±`MODULE_OFFSET_IN`), drive ratio 6.75, steer ratio 150/7, wheel radius half of `WHEEL_DIAMETER_IN`, CAN bus `drive`, IDs from `Ports`, encoder offsets 0 and all gains `PLACEHOLDER(tuning)`.
- `SwerveIOSim` follows `MapleSimSwerveDrivetrain` in 2910's 2026 code, on `Arena2025Reefscape`.
- `atTarget`: within 0.03 m and 2°. `setSpeedScale` scales teleop speed and the drive-to-point cap.

- [ ] **Step 1: Write the failing tests** (all `@Tag("sim")`)
```java
@Test void driveToPointArrives() { var target = Reef.scoringPose(Alliance.Blue, Reef.Branch.A); sim.setTruePose(StartPose.CENTRE.pose(Alliance.Blue)); drive.resetPose(sim.truePose()); drive.driveToPoint(target, 3.0, false); SimHarness.run(4); assertTrue(drive.atTarget()); assertEquals(0, sim.truePose().getTranslation().getDistance(target.getTranslation()), 0.05); }
@Test void speedScaleSlowsTheRobot() { /* full stick for 1.5 s at scale 1.0 and at 0.35: second run covers under half the distance */ }
@Test void teleopStickMovesFieldRelative() { /* +x stick from a 90-degree heading still moves in field +x */ }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*DriveSimTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement. Keep 2910's MIT header on `SwerveIOCTRE`, `SwerveIOSim` and `SwerveSubsystem`.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: swerve drivetrain with field simulation`.

### Task 14: Vision

**Files:**
- Create: `subsystems/vision/{VisionIO,VisionIOLimelight,VisionIOSim,Vision}.java`
- Test: `match/VisionSimTest.java`

**Interfaces:**
- Consumes: `SingleTagFix`, `CameraModel`, `TagObservation`, `SwerveSubsystem.resetTranslation`.
- Produces:
```java
public interface VisionIO { @AutoLog class VisionIOInputs { public boolean connected, hasTarget; public int tagId; public double txDeg, heightPx; } default void updateInputs(VisionIOInputs in) {} }
public class VisionIOLimelight implements VisionIO { public VisionIOLimelight(String name); }                 // reads tv, tid, tx, tcornxy from NetworkTables
public class VisionIOSim implements VisionIO { public VisionIOSim(Supplier<Pose2d> truePose, CameraModel cam); }
public class Vision extends SubsystemBase {
  public Vision(VisionIO io, SwerveSubsystem drive);
  public void setTargetFace(Optional<Integer> face);        // empty means no fixes
  public Optional<Translation2d> lastFix();
}
```
- `Vision` applies a fix only when a target face is set, the seen tag is that face's tag for the current alliance, and the fix is within 1.0 m of the current estimate. It logs `Vision/Fix` and `Vision/Rejected`.

- [ ] **Step 1: Write the failing tests** (all `@Tag("sim")`)
```java
@Test void fixCorrectsAnOdometryError() { placeAt(oneMetreFromFace(0)); drive.resetPose(shifted(sim.truePose(), 0.25, -0.15)); vision.setTargetFace(Optional.of(0)); SimHarness.run(0.3); assertEquals(0, drive.pose().getTranslation().getDistance(sim.truePose().getTranslation()), 0.05); }
@Test void ignoresTheWrongFace() { placeAt(oneMetreFromFace(0)); drive.resetPose(shifted(sim.truePose(), 0.25, 0)); vision.setTargetFace(Optional.of(3)); SimHarness.run(0.3); assertTrue(vision.lastFix().isEmpty()); }
@Test void noFixWhenNoFaceIsSet() { /* target face empty: pose error is unchanged after 0.3 s */ }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*VisionSimTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: single-camera reef position fix`.

### Task 15: Scoring commands, controls and device check

**Files:**
- Create: `commands/ScoreCommands.java`, `commands/DeviceCheck.java`, `sim/SimulatedRobot.java` (first version: steps every `simulate`, delivers coral, publishes poses)
- Modify: `RobotContainer.java`, `Robot.java`
- Test: `match/TeleopCycleTest.java`

**Interfaces:**
- Consumes: everything from Tasks 9 to 14.
- Produces:
```java
public final class ScoreCommands {
  public static final double RAISE_DISTANCE_M = Units.inchesToMeters(48);
  public static Command alignAndScore(RobotContainer r, Supplier<Reef.Branch> branch, Supplier<Superstructure.Wanted> level, double maxSpeedMps, boolean teleopGains);
  public static Command stationPickup(RobotContainer r, Supplier<Station.Side> side, double maxSpeedMps, boolean teleopGains);
  public static Command algaeFromReef(RobotContainer r, IntSupplier face);
  public static Command netScore(RobotContainer r);
}
public final class DeviceCheck { public static List<String> missing(); }        // names from Ports.all() that do not answer
public class RobotContainer { public SwerveSubsystem drive(); public Superstructure superstructure(); public Vision vision(); public SimulatedRobot sim(); /* null outside SIM */ }
public class SimulatedRobot { public void simulate(double dt); public Pose3d[] componentPoses(); public void preloadCoral(); public static final double HUMAN_DELAY_S = 0.7; }
```
- `alignAndScore`: set the vision target face; drive along `Reef.route` to the branch pose; once within `RAISE_DISTANCE_M` set the level; when `drive.atTarget()` and `superstructure.atPose()` request release; finish when the coral is gone or 1.0 s after the release request. On end or interrupt: clear the release, clear the vision face, set `STOW`.
- `stationPickup`: drive to `Station.pickupPose`, set `STATION_INTAKE`, finish when a coral is seen.
- Bindings follow the Driver controls table in the spec exactly. The drive's speed scale is fed from `superstructure.driveSpeedScale()` every loop.
- `SimulatedRobot` delivers a coral `HUMAN_DELAY_S` after the robot is within 0.15 m and 5° of a station pickup pose with `STATION_INTAKE` wanted. `componentPoses()` returns stage A, stage B, carriage, wrist: heights h/3, 2h/3, h, and the wrist at (`PIVOT_X_IN`, 0, `PIVOT_REST_HEIGHT_IN` + h) pitched by the wrist angle. Logged as `Mechanism/Components`.
- `Robot` logs `DeviceCheck/Missing` at start-up and raises one alert per missing device.

- [ ] **Step 1: Write the failing test** (`@Tag("sim")`)
```java
@Test void stationToL4Cycle() {
  startAt(Station.pickupPose(Alliance.Blue, Station.Side.RIGHT), noCoral());
  schedule(ScoreCommands.stationPickup(r, () -> Station.Side.RIGHT, 3.0, true)); SimHarness.run(2.0);
  assertTrue(r.superstructure().hasCoral());
  schedule(ScoreCommands.alignAndScore(r, () -> Reef.Branch.C, () -> Wanted.L4, 3.0, true)); SimHarness.run(6.0);
  assertFalse(r.superstructure().hasCoral());
  assertEquals(Wanted.STOW, r.superstructure().wanted());
}
@Test void deviceCheckIsCleanInSimulation() { assertTrue(DeviceCheck.missing().isEmpty()); }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*TeleopCycleTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the test. Expected: PASS.
- [ ] **Step 5:** Run `./gradlew simulateJava`, connect AdvantageScope, drive with a controller. Expected: the robot drives, both triggers align and score, `Mechanism/Components` moves.
- [ ] **Step 6:** Commit `feat: scoring commands, driver controls and device check`.

---

# Milestone 4: autos, game pieces and AdvantageScope

Ends with every auto passing on both alliances and the CAD model moving in AdvantageScope.

### Task 16: Game pieces and the score judge

**Files:**
- Create: `sim/ScoreJudge.java`
- Modify: `sim/SimulatedRobot.java`
- Test: `match/TeleopCycleTest.java` (add cases)

**Interfaces:**
- Produces:
```java
public final class ScoreJudge {
  public record CoralScore(Alliance alliance, Reef.Branch branch, Superstructure.Wanted level, double timeS) {}
  public static Optional<CoralScore> judgeCoral(Alliance a, Pose2d truePose, double carriageIn, double wristDeg, double timeS);
  public static boolean judgeNet(Alliance a, Pose2d truePose, double carriageIn, double wristDeg);
}
// SimulatedRobot additions
public List<ScoreJudge.CoralScore> coralScored(); public int algaeInNet(); public boolean reefAlgaePresent(Alliance a, int face);
```
- `judgeCoral` scores when the robot is within 1.5 in and 2° of a branch's scoring pose and the wrist axis line, from the pivot at (`PIVOT_X_IN`, `PIVOT_REST_HEIGHT_IN` + carriage) at the wrist angle, passes within 1.0 in of that level's branch point. Branch points in the robot frame, forward and up in inches: L2 (19.025, 31.875), L3 (19.025, 47.625), L4 (18.525, 72.0), L1 any pose within tolerance of `Pose.L1`. Each branch and level scores once.
- `SimulatedRobot` calls the judge on the tick the coral leaves, records the result, and spawns a maple-sim coral projectile from the wrist tip for the picture. An algae is taken when the claw is intaking, the pose matches the face's height and the robot is within 2 in of `Reef.algaePose`. `judgeNet` needs `Pose.NET` within tolerance and the front bumper within 0.3 m of the barge line (`PLACEHOLDER(field-check)`).
- Logs `Sim/TruePose`, `Sim/Coral`, `Sim/Algae`, `Sim/Score/Coral`, `Sim/Score/Net`.

- [ ] **Step 1: Write the failing tests** (`@Tag("sim")`)
```java
@Test void cycleScoresOnTheNamedBranch() { /* the Task 15 cycle, then: */ assertEquals(List.of(Reef.Branch.C), sim.coralScored().stream().map(CoralScore::branch).toList()); assertEquals(Wanted.L4, sim.coralScored().get(0).level()); }
@Test void releaseAwayFromTheReefDoesNotScore() { seatCoral(); r.superstructure().setWanted(Wanted.L4); SimHarness.run(2.5); r.superstructure().requestRelease(true); SimHarness.run(0.5); assertTrue(sim.coralScored().isEmpty()); }
@Test void algaeComesOffTheFace() { schedule(ScoreCommands.algaeFromReef(r, () -> 3)); SimHarness.run(5); assertTrue(r.superstructure().hasAlgae()); assertFalse(sim.reefAlgaePresent(Alliance.Blue, 3)); }
```
- [ ] **Step 2:** Run `./gradlew simTest --tests '*TeleopCycleTest'`. Expected: the new cases FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the tests. Expected: PASS.
- [ ] **Step 5:** Commit `feat: game piece simulation and score judge`.

### Task 17: Autos

**Files:**
- Create: `autos/{Auto,AutoProgram,AutoChooser,AutoFactory}.java`
- Modify: `Robot.java`
- Test: `match/AutoMatchTest.java`

**Interfaces:**
- Consumes: `ScoreCommands`, `StartPose`, `Reef`, `Station`, `SimulatedRobot.coralScored`.
- Produces:
```java
public enum Auto { IDLE, LEAVE, ONE_RIGHT, ONE_CENTRE, ONE_LEFT, THREE_RIGHT_ECD, THREE_LEFT_JLK, CENTRE_H_ALGAE }
public record AutoRoutine(Optional<Pose2d> startPose, Command command, List<Reef.Branch> expectedCoral) {}
public record AutoProgram(Auto auto, String label, Function<AutoFactory, AutoRoutine> build) {}
public class AutoFactory { AutoFactory(Alliance a, RobotContainer r); /* one package-private method per Auto, returning AutoRoutine */ }
public class AutoChooser {
  public static final List<AutoProgram> PROGRAMS;
  public AutoChooser(RobotContainer r);
  public void update();                                   // call in disabledPeriodic: builds the selection for both alliances
  public Auto selected(); public void select(Auto a);     // select() is for tests
  public Optional<Command> selectedCommand(); public Optional<Pose2d> startPose();
  public void setPoseToStart(); public boolean poseSetForSelected();
}
```
- Labels are the "Name on the dashboard" column of the spec, verbatim. Sequences are the "What it does" column. Scoring legs run at 3.0 m/s, station legs at 3.0 m/s, all `PLACEHOLDER(tuning)`.
- A station leg waits at most 2.0 s at the station for a coral, then moves on. A scoring leg skipped for want of a coral goes straight to the next station leg.
- `selectedCommand()` is empty when the alliance is unknown, and `Robot.autonomousInit` then raises alert "No alliance: auto not run".
- The chooser and a "Set pose to auto start" button publish under `SmartDashboard/Auto`. `poseSetForSelected()` goes false whenever the selection or alliance changes. Logs `Auto/Selected` and `Auto/PoseSet`.
- `CENTRE_H_ALGAE` after scoring H: `algaeFromReef` on face 3, drive to the net line, `netScore`.

- [ ] **Step 1: Write the failing tests** (`@Tag("sim")`)
```java
@ParameterizedTest @MethodSource("everyAutoOnBothAlliances")
void autoScoresWhatItClaims(Auto auto, Alliance alliance) {
  var h = AutoHarness.start(auto, alliance);                 // sets alliance, selects, runs update(), presses the pose button, preloads, enables auto
  SimHarness.run(15.0);
  assertEquals(h.routine().expectedCoral(), h.sim().coralScored().stream().map(CoralScore::branch).toList());
  assertTrue(h.sim().coralScored().stream().allMatch(s -> s.timeS() <= 15.0));
  assertTrue(h.keepOutNeverViolated());
  assertTrue(h.chooser().poseSetForSelected());
}
@Test void startPoseMatchesTheRoutine() { /* after the button: drive.pose() equals routine.startPose() for every auto with one */ }
@Test void unknownAllianceRunsNothing() { setAlliance(Optional.empty()); chooser.select(Auto.ONE_CENTRE); chooser.update(); assertTrue(chooser.selectedCommand().isEmpty()); }
@Test void allianceSwitchAfterBuildUsesTheRightSide() { /* build as Blue, switch to Red, run ONE_CENTRE: coral scores on Red's H */ }
@Test void poseFlagClearsWhenSelectionChanges() { chooser.select(Auto.ONE_LEFT); chooser.setPoseToStart(); chooser.select(Auto.ONE_RIGHT); assertFalse(chooser.poseSetForSelected()); }
@Test void deadCoralSensorEndsLegsOnTimeout() { coralSim.setRangeConnected(false); var h = AutoHarness.start(Auto.THREE_RIGHT_ECD, Alliance.Blue); SimHarness.run(15.0); assertFalse(h.commandStillRunningALeg()); }
```
  `expectedCoral`: IDLE and LEAVE empty; ONE_RIGHT [E]; ONE_CENTRE [H]; ONE_LEFT [J]; THREE_RIGHT_ECD [E, C, D]; THREE_LEFT_JLK [J, L, K]; CENTRE_H_ALGAE [H]. `CENTRE_H_ALGAE` also asserts `algaeInNet() == 1`.
- [ ] **Step 2:** Run `./gradlew simTest --tests '*AutoMatchTest'`. Expected: FAIL.
- [ ] **Step 3:** Implement, adapting 2910's 2025 `autos` package and keeping its MIT header.
- [ ] **Step 4:** Run the tests. Expected: PASS. If a three-coral routine cannot finish inside 15 s in simulation, raise leg speeds first; if it still cannot, report the measured time and stop for a decision. Do not loosen the test.
- [ ] **Step 5:** Commit `feat: selectable autos`.

### Task 18: AdvantageScope model and layout

**Files:**
- Create: `tools/step_to_glb.py`, `ascope/Robot_RobotA/{model.glb,model_0.glb,model_1.glb,model_2.glb,model_3.glb,config.json}`, `ascope/layout.json`

**Blocked until** the Robot A CAD folder is connected. Needs `robotA_assembly_complete.step`.

**Interfaces:**
- Consumes: `Mechanism/Components` (stage A, stage B, carriage, wrist, in that order), `RobotConstants.CameraK`.
- Produces: `python3 tools/step_to_glb.py <assembly.step> ascope/Robot_RobotA`.

- [ ] **Step 1:** Write `step_to_glb.py`: load the STEP, convert inches to metres, sort solids into base, stage A, stage B, carriage and wrist by their assembly group, and export `model.glb` (base) and `model_0` to `model_3`. Print each group's solid count and bounding box.
- [ ] **Step 2:** Run it. Expected: five files, every solid assigned to exactly one group, base bounding box about 0.71 m square.
- [ ] **Step 3:** Write `config.json`: `"name": "Robot A"`, rotations and position that put the model's origin on the carpet at robot centre with X forward, one fixed camera named "Limelight 4" at the nose (82° field of view, 1280 × 800), and four components. Stages and carriage have zero offsets; the wrist's `zeroedPosition` moves its pivot to the origin.
- [ ] **Step 4:** In AdvantageScope choose Use Custom Assets Folder and pick `ascope/`. Confirm the 2025 field is in the 3D Field tab's field list; if it is not, download it from AdvantageScope's asset list first. Run `./gradlew simulateJava` and add a 3D Field tab with `Sim/TruePose` as the robot and `Mechanism/Components` as its components. Expected: the stowed model matches the CAD, and raising to L4 moves three stages and tilts the wrist with no part left behind.
- [ ] **Step 5:** Add the ghost robot for the estimated pose, `Sim/Coral` and `Sim/Algae` as game pieces, a swerve tab, and line graphs of target against measured carriage height and wrist angle. Export the layout to `ascope/layout.json`.
- [ ] **Step 6:** Commit `feat: AdvantageScope robot model and layout`.

### Task 19: Wiring document, README and full run

**Files:**
- Create: `docs/wiring.md`, `README.md`
- Modify: `docs/open-numbers.md`

- [ ] **Step 1:** Write `docs/wiring.md` from the spec's CAN IDs and Wiring sections, tables verbatim.
- [ ] **Step 2:** Write `README.md`: how to build, run the simulator, open AdvantageScope with `ascope/`, run `test` and `simTest`, and add an auto.
- [ ] **Step 3:** Update `docs/open-numbers.md` so it lists every `PLACEHOLDER` in the code. Check with `grep -rn "PLACEHOLDER(" src/main/java | wc -l` against the document's count.
- [ ] **Step 4:** Run `./gradlew build simTest`. Expected: `BUILD SUCCESSFUL`, zero failed tests.
- [ ] **Step 5:** Commit `docs: wiring map, README and open numbers`.
