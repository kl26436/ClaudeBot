# Code decisions and math log

Source material for the playbook's Code it and Wire it chapters, its calculators and What went wrong. It is
the code side of the design log in the playbook repo (`docs/decisions-and-math.md`) and uses the same layout
and labels. Covers Nov 21 to 23, 2026.

**How sure are we.** Nothing here has run on a robot.
- *calculated*: worked by hand or in a script;
- *modelled*: measured on the CAD model;
- *simulated*: run in the robot code simulator, against a physics model;
- *assumed*: a guess that still needs measuring.

**Who decided.** "Kevin" marks a call Kevin made. "Code" marks a call made while writing the code that Kevin
has not overruled.

Motor data in the code and simulator: Kraken X60 with FOC, 5,800 rpm free, 9.37 N·m and 483 A at stall, so
0.0194 N·m per amp and 12 / 483 = 0.0248 Ω. (The design log uses the trapezoidal numbers.)

---

## 1. Decisions, in date order

### Nov 21: Real physics in the simulator (Kevin, from three options)
- **Decided:** Team 2910's project layout, with one motor class per mechanism that both the robot and the simulator run. In simulation, CTRE's simulated Krakens push a physics model that pushes back.
- **Options:** AdvantageKit's template, with separate real and simulated classes, which replays logs best but tests different control code from the robot's; or 2910's simulation as published, where mechanisms jump to their target, so a test of timing or an interlock means little.
- **Why:** a test is only worth having if the thing it tests takes time to move, can miss, and runs the robot's own code.

### Nov 21: No paths in auto (Kevin, approved with the design)
- **Decided:** every leg drives straight to a pose under a speed cap. A leg that would cross the reef goes through a via point on a 2.0 m circle round it. Red is blue turned 180° about field centre.
- **Why:** autos share the driver's own scoring code, so one thing is tuned and tested once. Every leg ends on a sensor or a timeout, so a late coral costs one piece and not the routine.
- **Option not taken:** PathPlanner or Choreo paths, left out of this pass.

### Nov 21: Keep-out rules are data, and one pure function keeps them (Code)
- **Decided:** the table of allowed wrist angles by carriage height lives in one file. One function with no hardware in it, `SuperstructurePlanner.next()`, turns where both axes are and where they should end up into safe targets, fifty times a second.
- **Why:** a function with no hardware can be tested with every pair of poses in a second. Only the superstructure may command a mechanism, so nothing can go round it.

### Nov 21: Use one camera and one tag for position near the reef (Kevin, approved with the design)
- **Decided:** near the reef, one tag plus the gyro heading gives the robot's position, and it overwrites the odometry position. Only the tag on the face being scored on counts.
- **Why:** once the heading is known, the angle to one tag and its height in pixels are enough. Team 2910 does it this way.
- **Cost:** no fix at the station, because the camera looks forward.

### Nov 24: A far-off fix has to prove itself (Code, found by the playbook's own replay)
- **Problem:** a fix more than 1 m from the estimate was always refused, so a robot that had been knocked 1.26 m out drove the right route in the wrong place and let its coral go onto the floor. Recording it for the playbook is how it was seen.
- **Decided:** a bad sighting jumps about; a wrong estimate stays wrong by the same amount. So a far fix that says the same thing, within 0.25 m, for 0.3 s without a break is believed. The route is planned again from there, and the release waits (at most 0.6 s) while the camera disputes the position.
- **Still true:** with no tag in view there is nothing to correct it, and the robot goes by its wheels.
- **Tests:** sightings that jump about are never believed, nor bursts shorter than 0.3 s; the 1.26 m start now scores.

### Nov 21: Simulate the camera from geometry (Code)
- **Decided:** a simulated camera works out what the Limelight would report from the robot's true pose, with noise, and the real position code runs on that. No extra vision library.

### Nov 22: The wrist is judged where it will stop, not where it is (Code, after a review)
- **Problem:** changing the goal in the middle of a move let the wrist swing into a height the carriage was still sliding through.
- **Decided:** each axis is checked over everywhere it will pass before it can stop. See 2.3.
- **Test:** every move between every three poses, interrupted every 60 ms, in a motion model. It fails without the fix.

### Nov 22: Limit acceleration by carriage height (Code, from drawing R-001)
- **Decided:** every commanded change of drive velocity stays within 80% of the tipping acceleration at the present carriage height. The 80% is assumed.

### Nov 22: What to do when a sensor dies (Code, after a review)
- **Coral sensor offline:** it reads "no coral", and the score pushes for a full second instead of skipping. Simulated: the three-coral auto still scores its preload at 2.3 s.
- **Any piece sensor offline:** the robot remembers the last piece it knew about, so an algae aboard still keeps the wrist up.
- **Why:** "offline" and "empty" look the same on a wire and mean opposite things to an interlock.

### Nov 23: The unstow band, 0.75 to 2.0 in (Code, measured on CAD package rev3)
- **Problem:** the drawing's table has no height where the wrist may leave stow. The clash grid showed every angle clear at exactly 1.25 in, and no mechanism holds exactly anything.
- **Measured:** 550,000 points on the wrist and 2.2 million on everything else, nearest distance at 12 heights and 19 angles. From 0.75 to 2.0 in, every angle from −90° to +75° clears by about a third of an inch or more.
- **Decided:** the wrist may be anywhere in that band; the carriage aims for 1.25 in, and the rest is room to miss. See 2.4.
- **Rechecked on packages 3.2 and 3.3** (with both mast braces): the same.
- **Check by hand on the robot first.**

### Nov 23: Phoenix counts rotor turns, the code converts to inches (Code)
- **Problem:** see "Elevator speed" in section 3.

### Nov 23: The wrist motor counts for itself, set from the encoder (Code, forced by the wiring)
- **Decided:** the absolute encoder goes to the roboRIO, so the motor controller cannot fuse it. The motor closes its loop on its own rotor, and the code sets that count from the encoder at start-up, after a motor reboot, and when a still wrist is more than 0.5° out.
- **Trusted only while** the two agree within 5°. Corrections that add up to more than 5° one way are a slipping encoder, and stop the wrist.

### Nov 23: The climb is one held button, and remembers itself (Code, from drawings CL-000 and R-001)
- **Decided:** hold to deploy, back onto the cage, hold again to wind in. Letting go at any point stops the winch with the pawls down.
- **The funnel can open once per match,** so it needs the button held 0.7 s in the climb pose and, on a field, the last 30 s.
- **The robot writes down that a climb has begun.** See "Hanging or stowed" in section 3.
- **The pawls lift only after the rope has held still for 0.3 s with under 20 A,** so a hanging robot cannot be let down. A hanging robot takes about 47 A (design log 2.5).

### Nov 23: A dead wrist sensor does not cost the climb (Kevin: "whatever is smartest")
- **Decided:** once a climb has begun the wrist only has to hold still, so it holds on its motor's count if the encoder is lost.

---

## 2. The math, worked with the robot's numbers

### 2.1 Elevator voltages: hold, cruise, speed up
- **Inputs (modelled):** stage A 5.71 lb, stage B 4.68 lb, carriage with wrist and claw 19.83 lb. One motor turn moves the carriage 22 × 0.25 × 3 / 12 = **1.375 in**.
- **Hold (kG):** 7.38 N·m at the shaft (design log 2.3) / 12 / 2 motors = 0.31 N·m each. 0.31 / 0.0194 = 15.8 A, × 0.0248 Ω = **0.39 V**. At 85% gearbox efficiency, 0.46 V.
- **Cruise (kV):** 12 V / (5,800 / 60 turns per second) = **0.124 V per turn per second**. At 110 in/s the motor turns 110 / 1.375 = 80 times a second, so 80 × 0.124 = **9.9 V**. Add kG and it is 10.3 V of the battery's 12: there is little left, which is why cruise is 80% of free speed.
- **Speed up (kA):** the weight as felt at the carriage is (5.71 + 2 × 4.68 + 3 × 19.83) / 3 = 24.9 lb. At 0.55 g that takes 0.17 N·m a motor, 8.7 A, 0.22 V, for 154 turns per second per second: **0.0014 V**.
- **Teaching point:** three numbers that can be worked out before the robot exists get the mechanism most of the way. Tuning only fixes what is left.

### 2.2 Motion profile: how long a move takes
- **Inputs:** 65.25 in of travel, 110 in/s cruise, 212 in/s² (0.55 g).
- Speeding up takes 110 / 212 = 0.52 s and covers 110² / (2 × 212) = **28.5 in**. Slowing down is the same.
- That leaves 65.25 − 57 = 8.25 in at cruise: 0.075 s. **Total 1.11 s** (calculated).
- **Any move shorter than 57 in never reaches cruise.** Its time is 2 × √(distance / acceleration): 17.5 in to L2 takes 2 × √(17.5 / 212) = 0.57 s.
- **Why 0.55 g:** nothing pulls the carriage down, so past 1 g the ropes go slack. The drawing's cap is 0.6 g; the profile asks for a little less so the carriage, catching up with it, stays near the cap.

### 2.3 Stopping distance: why the planner looks ahead
- **Rule:** distance to stop = speed × one loop (0.02 s) + speed² / (2 × deceleration).
- **Carriage at 110 in/s:** 2.2 + 28.5 = **30.7 in**. The whole band where the funnel limits the wrist is 23 in tall.
- **Wrist at 400 °/s and 1,600 °/s²:** 8 + 50 = **58°**.
- So a planner that looks only at where things are will be wrong by up to 31 in or 58°.

### 2.4 Tolerance: why a band, and how wide
- An axis with no integral term settles where friction and its gravity error are balanced: error = voltage error / kP. For the elevator kP is 0.7 V a turn, 0.51 V an inch, so **0.13 V of error is 0.25 in**.
- The first version needed the carriage within 0.25 in before the wrist could leave stow. 0.13 V of friction would have kept the robot in stow all match.
- **Rule:** wherever one thing waits for another, the window has to be wider than both tolerances added up. The test lets each axis settle anywhere inside its own tolerance (0.45 in, 1.4°) and requires every move to finish.

### 2.5 Drive acceleration cap
- Cap = 0.8 × tipping acceleration × 9.81 m/s². Stowed: 0.8 × 1.06 × 9.81 = **8.3 m/s²**. At the top: 0.8 × 0.50 × 9.81 = **3.9 m/s²**. Between the drawing's heights it is interpolated.
- The code uses 0.50 g at the top, the lowest of the drawing's two full-height poses (wrist at +60°); the design log's table shows 0.52 g for the wrist level.
- **Simulated:** driving off from a score at full stick with the carriage high, the robot's true acceleration peaked at 1.09 times the cap.

### 2.6 Can 60 A motors reach that?
- 60 A × 0.0194 = 1.16 N·m, × 6.75 gearing / 0.0501 m wheel radius = 157 N a wheel, 627 N in all. On 143.8 lb (65.2 kg) that is **9.6 m/s²**, a little over the stowed cap (calculated, no losses).

### 2.7 Drive to a point
- Speed = kP × distance + a little for friction, no more than the cap. With kP = 3.0 and a 3 m/s cap the robot starts slowing 1.0 m out.
- At the target: within 3 cm and 2°.

### 2.8 Auto time budget (simulated)

| Auto | First | Second | Third |
| --- | --- | --- | --- |
| 3 Right, blue | E at 2.4 s | C at 8.3 s | D at 13.6 s |
| 3 Left, blue | J at 2.4 s | L at 8.2 s | K at 13.6 s |

A station cycle takes about 5.6 s, of which 0.7 s is the simulated human player (assumed). Times vary by about 0.2 s from run to run. That leaves about 1.4 s of the 15.

### 2.9 Climb: how far is half a tooth
- A tooth is 3.2° of fork (design log 2.6; 3.6° with the cut wheel before package 3.3). A pawl resting on a tooth has to be unloaded before a servo can lift it, so the winch winds in first. Wind in a whole tooth and the pawl drops over the next one. **Half a tooth, 1.6°, is as far as it gets from both.**

---

## 3. Misses and what they taught (chapter 10 material)

- **Elevator speed: fixed Nov 23.** At 110 in/s the simulated profile stopped slowing down and the carriage hit the top, so it ran at 65. Gains, limits and step rates made no difference. The cause was doing the unit conversion inside the motor controller with a ratio below 1; counting rotor turns and converting in our own code gave a normal profile at once. It holds on the team PC, idle or with every core busy (simulated, Nov 24), but not in a cloud sandbox, where the simulated profile still stops dead at the target; why is not known. A real motor controller is neither, so raise the speed on the robot in steps. *Lesson: when nothing you tune changes a fault, stop tuning and change the structure.*
- **Hanging or stowed: found in review Nov 23.** A hanging fork reads 88° and a stowed one 90°, and nothing measures the fork. If robot code had restarted while hanging, it would have come back thinking "stowed", and the next press of the climb button would have lowered the robot. It now writes down that a climb has begun. *Lesson: ask of every state, "what if the code starts here?"*
- **The planner that forgot about momentum: found in review Nov 22.** See 2.3. *Lesson: "where is it" is the wrong question for anything that takes time to stop.*
- **A dead sensor scored nothing: found in review Nov 22.** The three-coral auto skipped every score, because an offline sensor reads the same as an empty one.
- **A robot that could not leave stow: found in review Nov 23, before it was ever built.** See 2.4.
- **A test that passed by crashing: found Nov 23.** The test that the robot keeps its acceleration limit when backing away from the reef had a sign wrong, and drove into the reef. It passed most of the time anyway, and the numbers it gave were collisions. *Lesson: make sure a test can fail, and that it fails for the reason you think. Remove the fix and watch.*
- **The wrist that read level when it was stowed: found Nov 22.** With the whole robot running, the encoder's messages arrived late, and the motor fell back to reading 0°. It only showed with everything switched on.
- **A wind-in that reloaded the pawl: found by a test Nov 23.** See 2.9.
- **Flaky tests are findings.** Three of the faults above (the crash, the wrist and the pawl) first showed as a test that only failed some of the time.

**How faults were found.** Two independent reviews of finished, passing code found 1 critical and 5 important faults on Nov 22, and 1 critical and 7 important on Nov 23. Passing tests do not mean correct; they mean nobody has thought of the case yet.

---

## 4. For the playbook

### Corrections to pages already live

| Page | Says | Now |
| --- | --- | --- |
| Elevator, What went wrong; design log 2.2 and section 3 | The code runs at 65 in/s; a fix has been proposed and not tried | Fixed Nov 23. Runs at 110 in/s; 1.11 s for full travel (calculated). Keep the story, with the cause above |
| Elevator, the math | Full travel in about 0.6 s | True at cruise only. With speeding up and slowing down, 1.11 s: the profile calculator shows it |
| Design log, keep-out table | Every angle is clear at exactly 1.25 in | Clear from 0.75 to 2.0 in (modelled) |
| Design log 2.12 | Reef algae centres at 35.6 and 51.5 in | The code handoff and wrist drawing say 35.0 and 50.9. One is wrong; the code uses 35.0 and 50.9 |
| Design doc, What exists today | Code tasks 1 to 10 of 19 done | All done, plus the climber |

### What Code it can hold, in the four-part template
1. **What we decided:** one motor class for robot and simulator; one planner that keeps the rules; autos made of the driver's own moves; every device in one file.
2. **Why:** section 1.
3. **The math:** 2.1 to 2.7, each a calculator. Four are ready in `calcs-code.ts` beside this file.
4. **Your turn:**
   - [ ] List every device with its ID in one file, and write a test that compares it with the wiring list.
   - [ ] Write each mechanism's safe range as a table. Put it in one function that a test can call.
   - [ ] Work out kG, kV and kA on paper before tuning.
   - [ ] Simulate it at full speed, with gravity, before trusting the paper number.
   - [ ] For every sensor, write down what the robot does when it dies.
   - [ ] For every fix, remove it and watch the test fail.

### Material ready to use
- **[`replays/`](replays/README.md):** every auto on blue and one climb, as really driven in simulation, ten rows a second. Its README has the format, the events and how to move the 3D model.
- **`ascope/Robot_ClaudeBot/`:** the real CAD model in seven pieces that move separately (about 5 MB in all, GLB, metres, Y up). With `replays/` it can play an auto in 3D. `SimulatedRobot.componentPoses()` has the position of each piece. Parts are merged by colour, so it cannot name a clicked part.
- **`docs/wiring.md`:** CAN map, power and bus order, for Wire it.

### Still missing
- **A current budget** for Wire it. The simulator's battery does not sag, so it cannot give one; it needs either a battery model or the robot.
- **Anything *tested*.**
