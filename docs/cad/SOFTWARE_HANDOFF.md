# ClaudeBot software handoff

> Copy of the claude.ai doc "Robot A software handoff" (exported Nov 23, 2026), placed here so Claude Code can read it.
> The two files it calls `lists/robotA_can_map.csv` and `lists/robotA_joints_and_limits.md` are in this folder, as `can_map.csv` and `joints_and_limits.md`.
> The claude.ai links below (code design, build instructions) open only in a browser signed in to claude.ai; Claude Code cannot read them.

Nov 23, 2026 · @Kevin

Everything the final CAD (November 23) asks of the code: what to fix first, the conventions for every joint, the setpoints, the wrist safety rule, the climb and funnel sequences, and the constants for odometry and simulation.

## Fix first

Five items, most urgent first. The first two are open problems already in the code; the rest are new work the finished CAD creates.

- [ ] **Wrist keep-out table and the unstow gap.** The code still uses the four-zone table; the CAD's six-zone table (below, in *Wrist keep-out rule*) is the one to use. Read literally it leaves no carriage height where the wrist may leave stow. The swept clash grid shows every wrist angle clear at exactly 1.25 in, so hold the carriage at 1.25 while the wrist leaves or returns to stow. Verify on the robot before trusting it.
- [ ] **Elevator cruise speed.** Held at 65 in/s (design 110). At 110 in/s in the Phoenix simulator the Motion Magic reference does not slow down properly and the carriage hits the top stop. Untried fix: keep Phoenix in rotor rotations and convert to inches in the IO class.
- [ ] **Climber and funnel latch.** Both are designed now and are not in the code yet: two winch motors, two pawl servos on one PWM, the latch servo, and the climb sequence below.
- [ ] **AdvantageScope robot model (task 18).** The CAD is split by motion group (mast, stage A, stage B, carriage, wrist, climber fork, funnel tray), with every pivot and axis in `joints_and_limits.md`. The meshes behind the 3D view on the kickoff page are a starting point.
- [ ] **Wrist sign in Onshape.** Anyone reading angles off the Onshape mate: mate 0 is stowed and minus raises the tip, so the design angle theta = -(mate + 90). The code uses theta.

## Where the code stands

As of the last code session (November 21): tasks 1 to 10 of 19 are done, with 59 unit tests and 35 real-time simulation tests passing. The code has not been touched since; this handoff comes from the CAD side.

- **Code:** `C:\Users\klape\Documents\ClaudeFRCBot`, branch `robot-a`, commit `7beb2d5`, not pushed. Start with `docs/HANDOFF.md`, then the plan `docs/superpowers/plans/2026-11-21-claudebot-code.md`.
- **Design:** [ClaudeBot code design](https://claude.ai/artifact/6YYpCCkQaGtHnapUWu4cRd) (approved November 21). It does not cover the climber, the funnel latch or the final keep-out table: this doc does.
- **Stack:** Java, WPILib 2026, AdvantageKit, Phoenix 6 Pro, Team 2910-style layout and autos, real physics in simulation.
- **Done:** scaffold, CAN map and constants, keep-out table (four zones), superstructure planner, field geometry, single-tag position fix, drive-to-point, elevator and wrist against physics.
- **Next:** 11 coral rollers and algae claw; 12 superstructure; 13 to 15 drivetrain, vision, scoring commands and controls; 16 and 17 game pieces and autos; 18 AdvantageScope model; 19 wiring doc, README, full run.
- **CAD sources:** drawing package `robotA_cad_package_rev3_2026-11-23` (sheet R-001 has the code rules; `lists/robotA_can_map.csv`; `lists/robotA_joints_and_limits.md`), and the Onshape document *Robot A - Full robot*. The wiring for the CAN IDs below is in [ClaudeBot build instructions](https://claude.ai/artifact/NBBpmCtf4Wyv4jgVFkgQrD), section 8.

## Joint conventions

One line per mechanism: what zero means, which way is positive, the hard limits and how the code knows where it is. Angles are in degrees, lengths in inches.

| Mechanism | Value and zero | Limits | Motors (CAN ID) | Reduction | Sensor |
| --- | --- | --- | --- | --- | --- |
| Elevator | Carriage height above rest; 0 = all stages down on their rest pads. Each stage moves the same distance relative to its parent (cascade), so stage A = c/3. | 0 to 66; up-stops E-108 at the top | Kraken X60 51 left, 52 right: mirror images, one follows the other inverted | 12:1 gearbox, 22T #25 sprocket (5.5 in of chain per shaft turn) moving stage A; carriage moves 3x stage A: about 1.375 in of carriage per motor turn | Motor encoders; home switch on DIO 0 at the bottom |
| Wrist | theta: -90 = stowed, hanging between the rails; 0 = level, pointing forward; plus raises the tip | -93 and +77 (lugs W-210 on post W-301) | Kraken X60 53 | MAXPlanetary 16:1, then 16T to 60T #25 chain: 60:1 | REV through-bore absolute encoder on the pivot, DIO 1 (duty cycle) |
| Coral rollers | Surface speed in | - | Kraken X44 54 | 3:1 to the front wheel shaft, both shafts linked | CANrange 61 looks down into the wrist |
| Claw rollers | Front and rear turn opposite ways, both pulling the ball in | - | Kraken X44 55 front, 56 rear | 2:1 each | CANrange 62 looks along the mouth |
| Climber fork | psi: 90 = stowed, straight up; 0 = level, pointing back | Hard stops at -2 and 97; the climb ends at 88 in code | Kraken X60 57 left, 58 right: leader and follower, opposite directions, brake mode | MAXPlanetary 25:1 per side, rope onto a 3.5 in track: 156:1 overall | Motor encoders only (no fork angle sensor): start every match stowed |
| Climber pawls | Down = ratchet locks against paying out | 16 deg of lift (14 frees a tooth) | REV servos, both on PWM 1 | Horn 145 deg = pawls down, 123 deg = lifted | - |
| Funnel tray | 0 = closed and latched, 90 = open (swung up and over to the right) | One shot per match: no motor closes it | Latch servo on PWM 0 | Horn 10 deg up = closed, 32 deg down = open | - |
| Swerve | Standard MK4i | - | Drive 11 / 21 / 31 / 41, steer 12 / 22 / 32 / 42 (CANivore) | Team standard ratio (the weight model used L2) | CANcoders 13 / 23 / 33 / 43 |

The elevator reduction is worked from the drawings: check it against the constants already in the code before trusting either.

**Starting current limits** (sheet EL-001): drive 60 A stator, steer 30, lift 60, wrist 40, rollers 30, climber 60. Every Kraken channel has a 40 A breaker.

**Loads to expect:** wrist holding torque at worst about 110 lb-in empty, 124 with a coral, 137 with an algae. Climb: about 2,140 lb-in on the fork, 305 lb of rope per side, about 47 A per climber motor, 231 deg/s with no load.

## Setpoints

From the wrist and robot drawings (wrist and claw sheets 2 and 3, robot sheet R-001), with the bumper against the reef or wall. The wrist pivot is 24.5 in up plus the carriage height. All of these are first numbers to tune on a real field.

| Pose | Carriage (in) | Wrist pivot (in) | Wrist theta (deg) | Notes |
| --- | --- | --- | --- | --- |
| Stowed, match start | 0 | 24.5 | -90 | Inside the frame, 38.9 in tall |
| Station intake | 1.25 | 25.75 | -35 | Wrist floor 0.32 below the tray line: the coral steps down onto it |
| L2 | 17.5 | 42.0 | -35 | Coral travels 1.2 past the branch tip |
| L3 | 33.3 | 57.8 | -35 | Same angle, 15.8 higher |
| L4 | 66 | 90.5 | -53 | All the travel; coral travels 6.7 past the tip. Test early |
| Low algae | 8.8 | 33.3 | +46 | Ball on the L2 branch pair, centre 35.0 up; lower roller passes 0.82 under the branch tips |
| High algae | 24.6 | 49.1 | +46 | Ball on the L3 branch pair, centre 50.9 up |
| Processor | 1.25 | 25.75 | +15 | Bumper 3.7 off the wall, ball centre 17.5 up. Marked marginal on the drawings |
| Net | 66 | 90.5 | +75 | Ball bottom 92.8, 3.9 over a rim taken as 88.9 (confirm). Tallest point 104.6 at the front roller |
| Climb | 1.25 | 25.75 | 0 | Claw clear of the bay behind the elevator |

With the elevator at full height the tip angle on the field drops from 46.9 deg (stowed) to 27.5 deg: limit drive acceleration whenever the carriage is high.

## Wrist keep-out rule

The lowest wrist angle the code may allow at each carriage height (sheet R-001). Anything below it hits the elevator, the frame, the camera, the lift gearbox, the funnel or the stowed climber fork.

| Carriage height (in) | Lowest wrist angle allowed |
| --- | --- |
| Below 1.25 | Stowed (-90) only |
| 1.25 to 7 | -75 |
| 7 to 9 | -60 |
| 9 to 15 | -45 |
| 15 to 23 | -40 |
| 23 to 66 | Any angle |

This is the conservative form of a swept clash check: 3,256 exact tests on a grid of 15 carriage heights and 12 wrist angles, against funnel, fork, camera, every elevator stage at its own height, frame and modules. The check is a little looser than the table (for example at 17.5 to 22 in, stowed is clear again) but the code should keep the table.

**The unstow move.** Read literally, the table has no height where the wrist may leave stow: below 1.25 it must be at -90, and from 1.25 up it must be at -75 or higher. The clash check shows every angle clear at exactly 1.25. So:

1. With the wrist at -90, raise the carriage to 1.25 and hold it there.
2. Turn the wrist to -75 or higher.
3. Only then let the carriage move on. Stowing is the same in reverse.

The planner needs a small band around 1.25 where both are allowed, and a check that the carriage is inside it before the wrist moves. Verify the band on the robot by hand first.

**Never** turn the wrist with the carriage fully down, and never raise the carriage above 1.25 with the wrist stowed: above 1.25 the stowed claw meets the climber fork.

## Sequences

**Climb** (sheet CL-000 and R-001). Only in the endgame, and never reversible inside a match: the funnel cannot be closed again until the pit.

1. Carriage to 1.25, wrist to level (0). The claw leaves the bay behind the elevator.
2. Funnel latch servo (PWM 0) to open: the tubing throws the tray open to the right. Fire it only with the wrist already in the climb pose.
3. Wind in a little, then lift the pawls (PWM 1): a loaded pawl cannot lift. Pay out the winch: the torsion springs push the fork out of stow and its weight takes it to level, where it catches on the -2 deg stop.
4. The driver backs onto the cage until it sits on the two rubber sleeves; the gates snap shut behind its far pipes.
5. Pawls down, then wind in. The fork turns the cage top-first over the robot.
6. Stop near 88 deg and turn the motors off. The ratchets hold after the robot is disabled; the hard stop is at 97. The fork can settle back up to 3.6 deg (one tooth) before a pawl bites.

Hanging, the model says about 2.5 deg nose up, wheels 3.9 in off the carpet, bumper corners 5.4 in. The cage is held only by friction on the sleeves and gates: test on a real cage.

**Funnel latch.** One servo, one shot. Holding it closed is the hook spring's job, not the servo's: drive the servo to closed at enable and leave it there.

**Intake and release**

- Coral: carriage 1.25, wrist -35, coral rollers in until CANrange 61 sees the coral seated. Free speed at the wheels is 22 ft/s: run the intake at about a third.
- Algae: drive in with both claw rollers running; they touch the ball with the bumper still 0.50 in off the reef. Stop when CANrange 62 sees the ball. Free surface speed is 33 ft/s.
- Algae release (net or processor): reverse both claw rollers. The rollers let go after 1.5 in of travel, so the throw is weak until tested: plan to reach over the net rim, not to shoot from a distance.

## Robot constants for odometry and simulation

CAD axes match WPILib: X forward, Y left, Z up, origin at the robot centre on the carpet. Inches.

| Constant | Value | Source |
| --- | --- | --- |
| Frame | 28.0 x 28.0 | Drivebase sheet D-000 |
| Bumpers | 34.5 across, 2.0 to 7.0 off the carpet | Bumper sheet B-000 |
| Swerve wheel centres | (+-11.373, +-11.373): 2.627 in from each frame edge | SDS MK4i model |
| Wheel tread diameter | 3.947 as modelled (4 in nominal): measure the real wheel | SDS MK4i model |
| Mass | 115.6 lb robot; 129.9 with bumpers; 142.8 on the field with the battery | Robot sheet R-001 |
| Centre of gravity, stowed | (-1.16, -0.12, 9.55) on the field; (-0.18, -0.15, 10.68) robot only | Robot sheet R-001 |
| Camera, Limelight 4 | Body centre (13.0, 0.0, 9.45), optical axis pitched up 12 deg; lens on the front face, about 0.64 ahead of the body centre | Drivebase Rev F |
| Gyro, Pigeon 2 | Flat on the belly pan near the centre, arrow forward | Electronics EL-000 |
| Elevator plane | X = 4.5; wrist pivot at (4.5, 0, 24.5 + carriage height) | Elevator E-000 |
| Climber pivot | (-13.5, 0, 16.0), axis along Y | Climber CL-000 |
| Funnel hinge | Along X at Y = -7.1, Z = 27.5 | Funnel F-001 |
| Motors | 13 Kraken X60 (8 swerve, 2 lift, 1 wrist, 2 climber) and 3 Kraken X44 (coral rollers, two claw rollers) | CAN map |
| Extremes | Tallest 104.6 at the net pose; fork tip 15.2 behind the frame when level (limit 18) | Wrist and climber sheets |

For the AdvantageScope model, export one mesh per motion group from Onshape or from the package STEP, with each group's origin moved to its joint above.
