# Open numbers

Every value that is not yet confirmed carries a `// PLACEHOLDER(<reason>)` comment on its line.
This file lists each reason and what resolves it. Find the affected values with
`grep -rn "PLACEHOLDER(" src`.

| Reason | Meaning | What resolves it |
|---|---|---|
| `funnel-redesign` | Value depends on the funnel/intake geometry that is being redesigned. | The released CAD for the redesigned funnel. Replace the value with the measured dimension or setpoint. |
| `drawing` | Value should come from a mechanical drawing or CAD that does not exist yet. | The released drawing for that mechanism. Replace with the dimension read from it. |
| `tuning` | Value is a starting guess for a gain, limit or setpoint that must be found on the robot or in simulation. | A tuning session on the built robot. Replace with the tuned value and remove the comment. |
| `field-check` | Value depends on the real field's geometry or tolerances. | Measuring on a real field (or the official field drawings) at the first event or practice session. |

Rule: when a value is confirmed, change it and delete its `PLACEHOLDER` comment in the same commit.

## Known simulation limits

- **Elevator cruise speed.** `ElevatorK.CRUISE_IN_PER_S` is 65 in/s (about 1.3 s for full travel). The design value was 110 in/s. At 110 in/s in the Phoenix simulator the Motion Magic reference itself stops slowing down properly (its velocity falls from 110 to about 100 in/s, then at only about 12 in/s² instead of 231.65) and the carriage reaches the top hard stop. Below about 70 in/s the profile is normal. Gains, FOC, soft limits and the physics step rate make no difference. See `docs/HANDOFF.md`, Open problems. Do not raise the value on a real robot near the top of travel until this is understood.
- **CANrange dropout.** Phoenix simulates a CANrange's distance (`CANrangeSimState.setDistance`) but cannot take one off the bus. `CoralRollersIOSim.setRangeConnected` and `AlgaeClawIOSim.setRangeConnected` fake the dropout at the IO layer by reporting `rangeConnected = false`; the distance signal is still the simulated one.
- **Claw hold at stall.** Stalled on an algae, the 1.5 V claw hold voltage would draw about 45 A per motor, so the 30 A stator limit holds it to about 0.8 V. Intake (6 V) is limited the same way. Check on the robot that 30 A grips an algae firmly enough; if it does, the hold voltage could drop to about 0.8 V with no loss of grip.
- **Swerve FOC follows the licence.** CTRE's built-in field-centric requests run in native code and always ask the modules for FOC. `FieldSpeedsRequest` builds the module requests itself with `EnableFOC = PRO_LICENSED`, and `TunerConstants` picks `FusedCANcoder` or `RemoteCANcoder` the same way. CTRE's odometry thread decides time-synced sampling on its own; with `PRO_LICENSED = false`, check on the robot that no Pro fault appears on the drive bus.
- **Swerve in simulation.** `SwerveIOSim` gives maple-sim module constants with no offsets, inversions or steer-to-drive coupling (maple-sim models none), and its own steer gains. Each instance builds a fresh `Arena2025Reefscape`. After teleporting with `setTruePose`, wait a tick before `resetPose` so the Pigeon publishes its new yaw, or the estimate's heading is offset.
- **Odometry drift in simulation.** Straight driving and turning on the spot track the true pose to a few millimetres. Driving while turning drifts about 0.07 m over 1.5 s, from module steer lag and scrub that maple-sim models; the reef position fix corrects it.
- **Wrist remote-sensor frame rate.** The wrist motor closes its loop on the CANdi's PWM1 frames. At 50 Hz those frames went stale whenever CTRE's swerve odometry thread was running, and the motor read 0 degrees while the encoder read -90. They now run at 250 Hz, and `Wrist` distrusts any reading where the motor and the encoder disagree by more than 5 degrees. Check the CAN bus load on the real robot with everything running.
- **Robot-level tests share one robot.** Phoenix's simulator slows as devices are created and closed, so `TeleopCycleTest` (and the autos test) build one `RobotContainer` per class through `testutil/RobotFixture` and reset it between tests, instead of building one per test.
- **Lost sensors and lost positions.** A CANrange that stops answering reports "no piece" (so no auto leg waits on it), but the superstructure keeps the last known piece for its interlocks (algae carry, hold voltage) until a push out of `EJECT_CLEARS_S` clears it. A score with the coral sensor offline pushes for the full `RELEASE_TIMEOUT_S`. The elevator keeps a zero it finds at start-up above 1 in (a robot-code restart; an unpowered carriage falls to the bottom); if its motor controller reboots while running it stops, raises "Elevator lost its position", and waits for the carriage to be lowered by hand and the code restarted. Check on the robot that the Talon really keeps its position across a code restart.
- **Drive acceleration limit.** The drive holds every change of field velocity to `TIP_SAFETY` (80 %) of the tipping acceleration on drawing R-001 at the present carriage height: about 8.4 m/s^2 stowed, 4.0 m/s^2 at full height.

## Every placeholder in the code

Generated from the source on 2026-11-22. Re-check the count with `grep -rn "PLACEHOLDER(" src/main/java | wc -l`; a line with two reasons is one line.

82 lines in `src/main/java` carry a placeholder.

| Reason | Where | Value |
| --- | --- | --- |
| `drawing` | [RobotConstants.java:41](../src/main/java/frc/robot/config/RobotConstants.java#L41) | `COUPLING_RATIO = 50.0 / 14.0` |
| `drawing` | [RobotConstants.java:42](../src/main/java/frc/robot/config/RobotConstants.java#L42) | `BUMPER_FACE_FROM_CENTRE_IN = 17.4` |
| `drawing` | [RobotConstants.java:44](../src/main/java/frc/robot/config/RobotConstants.java#L44) | `ROBOT_MASS_LB = 140` |
| `drawing` | [RobotConstants.java:72](../src/main/java/frc/robot/config/RobotConstants.java#L72) | `STAGE_A_LB = 5` |
| `drawing` | [RobotConstants.java:73](../src/main/java/frc/robot/config/RobotConstants.java#L73) | `STAGE_B_LB = 4` |
| `drawing` | [RobotConstants.java:92](../src/main/java/frc/robot/config/RobotConstants.java#L92) | `MOI_KGM2 = 0.39` |
| `drawing` | [RobotConstants.java:105](../src/main/java/frc/robot/config/RobotConstants.java#L105) | `MOI_KGM2 = 0.0015` |
| `drawing` | [RobotConstants.java:115](../src/main/java/frc/robot/config/RobotConstants.java#L115) | `MOI_KGM2 = 0.003` |
| `drawing` | [RobotConstants.java:124](../src/main/java/frc/robot/config/RobotConstants.java#L124) | `FORWARD_IN = 12.0` |
| `drawing` | [Setpoints.java:10](../src/main/java/frc/robot/config/Setpoints.java#L10) | `L1(5, -35),` |
| `drawing` | [Setpoints.java:18](../src/main/java/frc/robot/config/Setpoints.java#L18) | `PROCESSOR(1.75, 15)` |
| `drawing` | [TunerConstants.java:65](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L65) | `STEER_INERTIA_KGM2 = 0.01` |
| `drawing` | [TunerConstants.java:66](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L66) | `DRIVE_INERTIA_KGM2 = 0.01` |
| `drawing`, `funnel-redesign` | [Setpoints.java:9](../src/main/java/frc/robot/config/Setpoints.java#L9) | `STATION(1.75, -30),` |
| `field-check` | [AutoFactory.java:42](../src/main/java/frc/robot/autos/AutoFactory.java#L42) | `LEAVE_DISTANCE_M = 1.0` |
| `field-check` | [Barge.java:20](../src/main/java/frc/robot/field/Barge.java#L20) | `Translation2d BLUE_NET_LINE = new Translation2d(8.25, 6.18)` |
| `field-check` | [Barge.java:23](../src/main/java/frc/robot/field/Barge.java#L23) | `NET_HALF_WIDTH_M = 1.8` |
| `field-check` | [StartPose.java:10](../src/main/java/frc/robot/field/StartPose.java#L10) | `LEFT(6.05),` |
| `field-check` | [StartPose.java:11](../src/main/java/frc/robot/field/StartPose.java#L11) | `CENTRE(4.026),` |
| `field-check` | [StartPose.java:12](../src/main/java/frc/robot/field/StartPose.java#L12) | `RIGHT(2.0)` |
| `field-check` | [StartPose.java:14](../src/main/java/frc/robot/field/StartPose.java#L14) | `BLUE_X_M = 7.2` |
| `field-check` | [ScoreJudge.java:43](../src/main/java/frc/robot/sim/ScoreJudge.java#L43) | `NET_LINE_TOL_M = 0.3` |
| `field-check` | [SwerveIOSim.java:47](../src/main/java/frc/robot/subsystems/drive/SwerveIOSim.java#L47) | `WHEEL_COF = 1.2` |
| `funnel-redesign` | [KeepOut.java:27](../src/main/java/frc/robot/config/KeepOut.java#L27) | `new Zone(6, 23, -38, 75),` |
| `funnel-redesign` | [Setpoints.java:11](../src/main/java/frc/robot/config/Setpoints.java#L11) | `L2(17.5, -35),` |
| `tuning` | [RobotContainer.java:56](../src/main/java/frc/robot/RobotContainer.java#L56) | `TELEOP_ALIGN_SPEED_MPS = 3.0` |
| `tuning` | [AutoFactory.java:26](../src/main/java/frc/robot/autos/AutoFactory.java#L26) | `SCORE_SPEED_MPS = 3.0` |
| `tuning` | [AutoFactory.java:27](../src/main/java/frc/robot/autos/AutoFactory.java#L27) | `STATION_SPEED_MPS = 3.0` |
| `tuning` | [AutoFactory.java:28](../src/main/java/frc/robot/autos/AutoFactory.java#L28) | `NET_SPEED_MPS = 2.5` |
| `tuning` | [AutoFactory.java:36](../src/main/java/frc/robot/autos/AutoFactory.java#L36) | `SCORE_LEG_TIMEOUT_S = 6.0` |
| `tuning` | [AutoFactory.java:37](../src/main/java/frc/robot/autos/AutoFactory.java#L37) | `STATION_LEG_TIMEOUT_S = 6.0` |
| `tuning` | [AutoFactory.java:38](../src/main/java/frc/robot/autos/AutoFactory.java#L38) | `ALGAE_LEG_TIMEOUT_S = 4.0` |
| `tuning` | [AutoFactory.java:39](../src/main/java/frc/robot/autos/AutoFactory.java#L39) | `NET_LEG_TIMEOUT_S = 4.0` |
| `tuning` | [ScoreCommands.java:33](../src/main/java/frc/robot/commands/ScoreCommands.java#L33) | `MIN_RELEASE_S = 0.25` |
| `tuning` | [ScoreCommands.java:55](../src/main/java/frc/robot/commands/ScoreCommands.java#L55) | `ALGAE_SPEED_MPS = 2.0` |
| `tuning` | [RobotConstants.java:68](../src/main/java/frc/robot/config/RobotConstants.java#L68) | `CRUISE_IN_PER_S = 65` |
| `tuning` | [RobotConstants.java:93](../src/main/java/frc/robot/config/RobotConstants.java#L93) | `CRUISE_DEG_PER_S = 400` |
| `tuning` | [RobotConstants.java:94](../src/main/java/frc/robot/config/RobotConstants.java#L94) | `ACCEL_DEG_PER_S2 = 1600` |
| `tuning` | [RobotConstants.java:103](../src/main/java/frc/robot/config/RobotConstants.java#L103) | `STATOR_LIMIT_A = 40` |
| `tuning` | [Superstructure.java:41](../src/main/java/frc/robot/subsystems/Superstructure.java#L41) | `FULL_SPEED_MAX_IN = 23.0` |
| `tuning` | [Superstructure.java:43](../src/main/java/frc/robot/subsystems/Superstructure.java#L43) | `MIN_SPEED_SCALE = 0.35` |
| `tuning` | [Superstructure.java:46](../src/main/java/frc/robot/subsystems/Superstructure.java#L46) | `TIP_SAFETY = 0.8` |
| `tuning` | [AlgaeClaw.java:10](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L10) | `INTAKE_VOLTS = 6` |
| `tuning` | [AlgaeClaw.java:11](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L11) | `HOLD_VOLTS = 1.5` |
| `tuning` | [AlgaeClaw.java:12](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L12) | `EJECT_VOLTS = -8` |
| `tuning` | [AlgaeClaw.java:15](../src/main/java/frc/robot/subsystems/claw/AlgaeClaw.java#L15) | `EJECT_CLEARS_S = 0.3` |
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
| `tuning` | [TunerConstants.java:48](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L48) | `SLIP_CURRENT_A = 120` |
| `tuning` | [TunerConstants.java:50](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L50) | `STEER_STATOR_LIMIT_A = 60` |
| `tuning` | [TunerConstants.java:59](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L59) | `FL_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:60](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L60) | `FR_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:61](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L61) | `BL_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:62](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L62) | `BR_OFFSET_ROT = 0.0` |
| `tuning` | [TunerConstants.java:67](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L67) | `STEER_FRICTION_V = 0.2` |
| `tuning` | [TunerConstants.java:68](../src/main/java/frc/robot/subsystems/drive/TunerConstants.java#L68) | `DRIVE_FRICTION_V = 0.2` |
| `tuning` | [ElevatorIOTalonFX.java:34](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L34) | `KG = 0.46` |
| `tuning` | [ElevatorIOTalonFX.java:35](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L35) | `KS = 0.0` |
| `tuning` | [ElevatorIOTalonFX.java:36](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L36) | `KV = 0.0903` |
| `tuning` | [ElevatorIOTalonFX.java:37](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L37) | `KA = 0.0012` |
| `tuning` | [ElevatorIOTalonFX.java:38](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L38) | `KP = 0.5` |
| `tuning` | [ElevatorIOTalonFX.java:39](../src/main/java/frc/robot/subsystems/elevator/ElevatorIOTalonFX.java#L39) | `KD = 0.01` |
| `tuning` | [WristIOTalonFX.java:33](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L33) | `KG = 0.30` |
| `tuning` | [WristIOTalonFX.java:34](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L34) | `KS = 0.0` |
| `tuning` | [WristIOTalonFX.java:35](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L35) | `KV = 7.45` |
| `tuning` | [WristIOTalonFX.java:36](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L36) | `KA = 0.05` |
| `tuning` | [WristIOTalonFX.java:37](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L37) | `KP = 60.0` |
| `tuning` | [WristIOTalonFX.java:38](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L38) | `KD = 1.0` |
| `tuning` | [WristIOTalonFX.java:41](../src/main/java/frc/robot/subsystems/wrist/WristIOTalonFX.java#L41) | `ENCODER_OFFSET_ROT = 0.0` |
