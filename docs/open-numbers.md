# Open numbers

Every value that is not yet confirmed carries a `// PLACEHOLDER(<reason>)` comment on its line.
This file lists each reason and what resolves it. Find the affected values with
`grep -rn "PLACEHOLDER(" src`.

| Reason | Meaning | What resolves it |
|---|---|---|
| `drawing` | Value should come from a mechanical drawing or CAD that does not exist yet. | The released drawing for that mechanism. Replace with the dimension read from it. |
| `tuning` | Value is a starting guess for a gain, limit or setpoint that must be found on the robot or in simulation. | A tuning session on the built robot. Replace with the tuned value and remove the comment. |
| `robot-check` | Value that only the built robot can show: a servo pulse, a direction, a clearance, a time nothing senses. | Checking it on the robot, by hand first. |
| `field-check` | Value depends on the real field's geometry or tolerances. | Measuring on a real field (or the official field drawings) at the first event or practice session. |

Rule: when a value is confirmed, change it and delete its `PLACEHOLDER` comment in the same commit.

## Known simulation limits

- **Elevator cruise speed.** 110 in/s, as designed. It was held at 65 because at 110 the Motion Magic reference in the Phoenix simulator stopped slowing down and the carriage reached the top stop. The cause was working in inches through `SensorToMechanismRatio` (a ratio below 1); with Phoenix in rotor rotations and the conversion in `ElevatorIOTalonFX` the profile is normal. The profile asks for 0.55 g so that the carriage, catching up with it, stays near the 0.6 g cap. On the robot, raise the speed in steps all the same.
- **CANrange dropout.** Phoenix simulates a CANrange's distance (`CANrangeSimState.setDistance`) but cannot take one off the bus. `CoralRollersIOSim.setRangeConnected` and `AlgaeClawIOSim.setRangeConnected` fake the dropout at the IO layer by reporting `rangeConnected = false`; the distance signal is still the simulated one.
- **Claw hold at stall.** Stalled on an algae, the 1.5 V claw hold voltage would draw about 45 A per motor, so the 30 A stator limit holds it to about 0.8 V. Intake (6 V) is limited the same way. Check on the robot that 30 A grips an algae firmly enough; if it does, the hold voltage could drop to about 0.8 V with no loss of grip.
- **Swerve FOC follows the licence.** CTRE's built-in field-centric requests run in native code and always ask the modules for FOC. `FieldSpeedsRequest` builds the module requests itself with `EnableFOC = PRO_LICENSED`, and `TunerConstants` picks `FusedCANcoder` or `RemoteCANcoder` the same way. CTRE's odometry thread decides time-synced sampling on its own; with `PRO_LICENSED = false`, check on the robot that no Pro fault appears on the drive bus.
- **Swerve in simulation.** `SwerveIOSim` gives maple-sim module constants with no offsets, inversions or steer-to-drive coupling (maple-sim models none), and its own steer gains. Each instance builds a fresh `Arena2025Reefscape`. After teleporting with `setTruePose`, wait a tick before `resetPose` so the Pigeon publishes its new yaw, or the estimate's heading is offset.
- **Odometry drift in simulation.** Straight driving and turning on the spot track the true pose to a few millimetres. Driving while turning drifts about 0.07 m over 1.5 s, from module steer lag and scrub that maple-sim models; the reef position fix corrects it.
- **Wrist angle.** The wrist motor closes its loop on its own rotor, seeded from the Through Bore encoder on DIO 1 at start-up, after a motor-controller reboot, and when a still wrist has drifted more than 0.5 degrees. More than 5 degrees apart for 0.1 s, or more than 5 degrees of corrections in one direction since start-up, is a fault: the wrist stops and the carriage holds. After a reboot there is about one loop in which the controller is wrong about where it is; it is in neutral until the next request, which is not sent until it has been seeded.
- **Unstow band.** `KeepOut` allows any wrist angle from 0.75 to 2.0 in; the planner aims for 1.25, and the rest is room for an axis that settles off its target. Measured on the rev3 assembly with surface samples every 0.1 in or closer: every angle from -90 to +75 clears the fixed parts by a third of an inch or more from 0.75 to 2.0 in. At 0.5 in the claw is 0.1 in from the lift gearbox at -65 degrees; a stowed wrist at -90 meets the climber fork from about 2.6 in, and one resting on its -93 stop from about 1.7 in. The same measurement agrees with drawing R-001 at every band edge; the tightest are 0.25 in at -40 degrees from 17.5 to 22 in and 0.26 in at -75 degrees at 7 in.
- **Climber.** Nothing measures the fork, the pawls or the tray. The fork angle is the rope the motor has wound, from stowed at power-up; the pawls and the tray are given a fixed time. In simulation a pawl that carries load does not lift and the ratchet holds to one tooth (3.6 degrees). Before the pawls lift, the rope has to hold still for 0.3 s with a mean motor current under 20 A; a hanging robot takes about 45 A. Check both currents on the robot. The cage is a constant worst-case torque, so nothing about capturing a real cage is tested.
- **Robot-level tests share one robot.** Phoenix's simulator slows as devices are created and closed, so `TeleopCycleTest` (and the autos test) build one `RobotContainer` per class through `testutil/RobotFixture` and reset it between tests, instead of building one per test.
- **Lost sensors and lost positions.** A CANrange that stops answering reports "no piece" (so no auto leg waits on it), but the superstructure keeps the last known piece for its interlocks (algae carry, hold voltage) until a push out of `EJECT_CLEARS_S` clears it. A score with the coral sensor offline pushes for the full `RELEASE_TIMEOUT_S`. The elevator keeps a zero it finds at start-up above 1 in (a robot-code restart; an unpowered carriage falls to the bottom); if its motor controller reboots while running it stops, raises "Elevator lost its position", and waits for the carriage to be lowered by hand and the code restarted. Check on the robot that the Talon really keeps its position across a code restart.
- **Drive acceleration limit.** The drive holds every change of field velocity to `TIP_SAFETY` (80 %) of the tipping acceleration on drawing R-001 at the present carriage height: about 8.4 m/s^2 stowed, 4.0 m/s^2 at full height.

## Every placeholder in the code

Generated from the source on 2026-11-23. Re-check the count with `grep -rn "PLACEHOLDER(" src/main/java | wc -l`; a line with two reasons is one line.

92 lines in `src/main/java` carry a placeholder.

| Reason | Where | Value |
| --- | --- | --- |
| `drawing` | [RobotConstants.java:41](../src/main/java/frc/robot/config/RobotConstants.java#L41) | `COUPLING_RATIO = 50.0 / 14.0` |
| `drawing` | [RobotConstants.java:81](../src/main/java/frc/robot/config/RobotConstants.java#L81) | `STAGE_A_LB = 5` |
| `drawing` | [RobotConstants.java:82](../src/main/java/frc/robot/config/RobotConstants.java#L82) | `STAGE_B_LB = 4` |
| `drawing` | [RobotConstants.java:102](../src/main/java/frc/robot/config/RobotConstants.java#L102) | `MOI_KGM2 = 0.39` |
| `drawing` | [RobotConstants.java:115](../src/main/java/frc/robot/config/RobotConstants.java#L115) | `MOI_KGM2 = 0.0015` |
| `drawing` | [RobotConstants.java:125](../src/main/java/frc/robot/config/RobotConstants.java#L125) | `MOI_KGM2 = 0.003` |
| `drawing` | [Setpoints.java:10](../src/main/java/frc/robot/config/Setpoints.java#L10) | `L1(5, -35),` |
| `drawing` | [TunerConstants.java:63](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L63) | `STEER_INERTIA_KGM2 = 0.01` |
| `drawing` | [TunerConstants.java:64](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L64) | `DRIVE_INERTIA_KGM2 = 0.01` |
| `field-check` | [AutoFactory.java:42](../src/main/java/frc/robot/autos/AutoFactory.java#L42) | `LEAVE_DISTANCE_M = 1.0` |
| `field-check` | [Barge.java:20](../src/main/java/frc/robot/field/Barge.java#L20) | `Translation2d BLUE_NET_LINE = new Translation2d(8.25, 6.18)` |
| `field-check` | [Barge.java:23](../src/main/java/frc/robot/field/Barge.java#L23) | `NET_HALF_WIDTH_M = 1.8` |
| `field-check` | [StartPose.java:10](../src/main/java/frc/robot/field/StartPose.java#L10) | `LEFT(6.05),` |
| `field-check` | [StartPose.java:11](../src/main/java/frc/robot/field/StartPose.java#L11) | `CENTRE(4.026),` |
| `field-check` | [StartPose.java:12](../src/main/java/frc/robot/field/StartPose.java#L12) | `RIGHT(2.0)` |
| `field-check` | [StartPose.java:14](../src/main/java/frc/robot/field/StartPose.java#L14) | `BLUE_X_M = 7.2` |
| `field-check` | [ScoreJudge.java:43](../src/main/java/frc/robot/sim/ScoreJudge.java#L43) | `NET_LINE_TOL_M = 0.3` |
| `field-check` | [SwerveIOSim.java:47](../src/main/java/frc/robot/subsystems/drive/SwerveIOSim.java#L47) | `WHEEL_COF = 1.2` |
| `robot-check` | [KeepOut.java:32](../src/main/java/frc/robot/config/KeepOut.java#L32) | `UNSTOW_MIN_IN = 0.75` |
| `robot-check` | [KeepOut.java:33](../src/main/java/frc/robot/config/KeepOut.java#L33) | `UNSTOW_MAX_IN = 2.0` |
| `robot-check` | [RobotConstants.java:183](../src/main/java/frc/robot/config/RobotConstants.java#L183) | `FUNNEL_OPEN_S = 1.0` |
| `robot-check` | [RobotConstants.java:190](../src/main/java/frc/robot/config/RobotConstants.java#L190) | `PAWL_LIFT_S = 0.4` |
| `robot-check` | [Setpoints.java:20](../src/main/java/frc/robot/config/Setpoints.java#L20) | `PROCESSOR(1.25, 15),` |
| `robot-check` | [ClimberIOTalonFX.java:44](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L44) | `PAWL_DOWN_US = (int) (1500 + PAWL_HALF_US)` |
| `robot-check` | [ClimberIOTalonFX.java:45](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L45) | `PAWL_LIFTED_US = (int) (1500 - PAWL_HALF_US)` |
| `robot-check` | [ClimberIOTalonFX.java:46](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L46) | `LATCH_CLOSED_US = (int) (1500 + LATCH_HALF_US)` |
| `robot-check` | [ClimberIOTalonFX.java:47](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L47) | `LATCH_OPEN_US = (int) (1500 - LATCH_HALF_US)` |
| `robot-check` | [ClimberIOTalonFX.java:72](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L72) | `config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive` |
| `tuning` | [RobotContainer.java:64](../src/main/java/frc/robot/RobotContainer.java#L64) | `TELEOP_ALIGN_SPEED_MPS = 3.0` |
| `tuning` | [AutoFactory.java:26](../src/main/java/frc/robot/autos/AutoFactory.java#L26) | `SCORE_SPEED_MPS = 3.0` |
| `tuning` | [AutoFactory.java:27](../src/main/java/frc/robot/autos/AutoFactory.java#L27) | `STATION_SPEED_MPS = 3.0` |
| `tuning` | [AutoFactory.java:28](../src/main/java/frc/robot/autos/AutoFactory.java#L28) | `NET_SPEED_MPS = 2.5` |
| `tuning` | [AutoFactory.java:36](../src/main/java/frc/robot/autos/AutoFactory.java#L36) | `SCORE_LEG_TIMEOUT_S = 6.0` |
| `tuning` | [AutoFactory.java:37](../src/main/java/frc/robot/autos/AutoFactory.java#L37) | `STATION_LEG_TIMEOUT_S = 6.0` |
| `tuning` | [AutoFactory.java:38](../src/main/java/frc/robot/autos/AutoFactory.java#L38) | `ALGAE_LEG_TIMEOUT_S = 4.0` |
| `tuning` | [AutoFactory.java:39](../src/main/java/frc/robot/autos/AutoFactory.java#L39) | `NET_LEG_TIMEOUT_S = 4.0` |
| `tuning` | [ClimbCommands.java:14](../src/main/java/frc/robot/commands/ClimbCommands.java#L14) | `ENDGAME_S = 30` |
| `tuning` | [ScoreCommands.java:33](../src/main/java/frc/robot/commands/ScoreCommands.java#L33) | `MIN_RELEASE_S = 0.25` |
| `tuning` | [ScoreCommands.java:55](../src/main/java/frc/robot/commands/ScoreCommands.java#L55) | `ALGAE_SPEED_MPS = 2.0` |
| `tuning` | [RobotConstants.java:77](../src/main/java/frc/robot/config/RobotConstants.java#L77) | `CRUISE_IN_PER_S = 110` |
| `tuning` | [RobotConstants.java:103](../src/main/java/frc/robot/config/RobotConstants.java#L103) | `CRUISE_DEG_PER_S = 400` |
| `tuning` | [RobotConstants.java:104](../src/main/java/frc/robot/config/RobotConstants.java#L104) | `ACCEL_DEG_PER_S2 = 1600` |
| `tuning` | [RobotConstants.java:157](../src/main/java/frc/robot/config/RobotConstants.java#L157) | `TENSION_LOAD_A = 20` |
| `tuning` | [RobotConstants.java:169](../src/main/java/frc/robot/config/RobotConstants.java#L169) | `CRUISE_DEG_PER_S = 60` |
| `tuning` | [RobotConstants.java:170](../src/main/java/frc/robot/config/RobotConstants.java#L170) | `ACCEL_DEG_PER_S2 = 240` |
| `tuning` | [RobotConstants.java:174](../src/main/java/frc/robot/config/RobotConstants.java#L174) | `STALL_A = 55` |
| `tuning` | [Superstructure.java:45](../src/main/java/frc/robot/subsystems/Superstructure.java#L45) | `FULL_SPEED_MAX_IN = 23.0` |
| `tuning` | [Superstructure.java:47](../src/main/java/frc/robot/subsystems/Superstructure.java#L47) | `MIN_SPEED_SCALE = 0.35` |
| `tuning` | [Superstructure.java:50](../src/main/java/frc/robot/subsystems/Superstructure.java#L50) | `TIP_SAFETY = 0.8` |
| `tuning` | [AlgaeClaw.java:10](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L10) | `INTAKE_VOLTS = 6` |
| `tuning` | [AlgaeClaw.java:11](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L11) | `HOLD_VOLTS = 1.5` |
| `tuning` | [AlgaeClaw.java:12](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L12) | `EJECT_VOLTS = -8` |
| `tuning` | [AlgaeClaw.java:15](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L15) | `EJECT_CLEARS_S = 0.3` |
| `tuning` | [ClimberIOTalonFX.java:32](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L32) | `KV = 19.4` |
| `tuning` | [ClimberIOTalonFX.java:33](../src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L33) | `KP = 720` |
| `tuning` | [CoralRollers.java:10](../src/main/java/frc/robot/subsystems/coral/CoralRollers.java#L10) | `INTAKE_VOLTS = 4` |
| `tuning` | [CoralRollers.java:11](../src/main/java/frc/robot/subsystems/coral/CoralRollers.java#L11) | `HOLD_VOLTS = 0.5` |
| `tuning` | [CoralRollers.java:12](../src/main/java/frc/robot/subsystems/coral/CoralRollers.java#L12) | `EJECT_VOLTS = -6` |
| `tuning` | [CoralRollers.java:15](../src/main/java/frc/robot/subsystems/coral/CoralRollers.java#L15) | `EJECT_CLEARS_S = 0.3` |
| `tuning` | [SwerveIOCTRE.java:21](../src/main/java/frc/robot/subsystems/drive/SwerveIOCTRE.java#L21) | `HEADING_KP = 7.0` |
| `tuning` | [SwerveIOCTRE.java:22](../src/main/java/frc/robot/subsystems/drive/SwerveIOCTRE.java#L22) | `HEADING_KD = 0.2` |
| `tuning` | [SwerveSubsystem.java:28](../src/main/java/frc/robot/subsystems/drive/SwerveSubsystem.java#L28) | `TELEOP_MAX_OMEGA_RAD_PER_S = 1.5 * 2 * Math.PI` |
| `tuning` | [SwerveSubsystem.java:31](../src/main/java/frc/robot/subsystems/drive/SwerveSubsystem.java#L31) | `DRIVE_TO_POINT_MAX_OMEGA_RAD_PER_S = 2 * Math.PI` |
| `tuning` | [TunerConstants.java:31](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L31) | `.withKP(100)` |
| `tuning` | [TunerConstants.java:33](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L33) | `.withKD(0.5)` |
| `tuning` | [TunerConstants.java:34](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L34) | `.withKS(0.1)` |
| `tuning` | [TunerConstants.java:35](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L35) | `.withKV(2.66)` |
| `tuning` | [TunerConstants.java:41](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L41) | `.withKP(0.1)` |
| `tuning` | [TunerConstants.java:44](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L44) | `.withKS(0)` |
| `tuning` | [TunerConstants.java:45](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L45) | `.withKV(0.124)` |
| `tuning` | [TunerConstants.java:48](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L48) | `SLIP_CURRENT_A = DriveK.DRIVE_STATOR_LIMIT_A` |
| `tuning` | [TunerConstants.java:57](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L57) | `FL_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:58](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L58) | `FR_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:59](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L59) | `BL_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:60](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L60) | `BR_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:65](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L65) | `STEER_FRICTION_V = 0.2` |
| `tuning` | [TunerConstants.java:66](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L66) | `DRIVE_FRICTION_V = 0.2` |
| `tuning` | [ElevatorIOTalonFX.java:33](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L33) | `KG = 0.46` |
| `tuning` | [ElevatorIOTalonFX.java:34](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L34) | `KS = 0.0` |
| `tuning` | [ElevatorIOTalonFX.java:35](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L35) | `KV = 0.124` |
| `tuning` | [ElevatorIOTalonFX.java:36](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L36) | `KA = 0.0017` |
| `tuning` | [ElevatorIOTalonFX.java:37](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L37) | `KP = 0.7` |
| `tuning` | [ElevatorIOTalonFX.java:38](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L38) | `KD = 0.014` |
| `tuning` | [Wrist.java:39](../src/main/java/frc/robot/subsystems/wrist/Wrist.java#L39) | `RESEED_DEG = 0.5` |
| `tuning` | [WristIOTalonFX.java:31](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L31) | `KG = 0.30` |
| `tuning` | [WristIOTalonFX.java:32](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L32) | `KS = 0.0` |
| `tuning` | [WristIOTalonFX.java:33](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L33) | `KV = 7.45` |
| `tuning` | [WristIOTalonFX.java:34](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L34) | `KA = 0.05` |
| `tuning` | [WristIOTalonFX.java:35](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L35) | `KP = 60.0` |
| `tuning` | [WristIOTalonFX.java:36](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L36) | `KD = 1.0` |
| `tuning` | [WristIOTalonFX.java:39](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L39) | `ENCODER_OFFSET_ROT = 0.0` |
| `tuning` | [WristIOTalonFX.java:42](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L42) | `ENCODER_SIGN = 1.0` |
