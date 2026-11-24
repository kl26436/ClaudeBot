# Simulated runs, for animating

Recordings of the robot code running in simulation, ten rows a second (the driver cycle and the vision runs twenty, the elevator move fifty): where the robot and its mechanisms
really were, not what its sensors thought. Small enough to load in a page (3 to 16 KB each).

| File | What it is |
| --- | --- |
| `leave.json`, `one_right.json`, `one_centre.json`, `one_left.json` | Leave, and one coral from each start |
| `three_right_ecd.json`, `three_left_jlk.json` | The three-coral autos |
| `centre_h_algae.json` | A coral, then the algae off that face and into the net |
| `climb.json` | One deep climb, from the climb button to hanging |
| `teleop_cycle.json` | A driver cycle: one command picks up at the right station, one scores on branch D at L4 |
| `vision_fix.json` | The position starts 0.43 m wrong; the camera puts it right |
| `vision_rejected.json` | The position starts 1.26 m wrong; the camera's fix is refused for 0.3 s, then believed |
| `elevator_move.json` | Stow to L4 and back at every 20 ms loop, with volts and amps |

All are blue. Label them *simulated*.

Regenerate them, after any change to the code, with

```
./gradlew simTest --tests '*AutoMatchTest*' --tests '*ReplayTest*' -PreplayDir=docs/playbook/replays
```

From another site, copy them at build time, or fetch
`https://raw.githubusercontent.com/kl26436/ClaudeBot/main/docs/playbook/replays/<file>`.

## The format

Every file starts with a label, `code` (the commit of this repo the run came from) and `recorded` (the date). If the
code has changed since `code` in a way that matters, record it again. `columns` names each value in a row of `samples`.

`events` are moments to mark. Each has `t`, in seconds, and `what`:

| `what` | In | Other fields |
| --- | --- | --- |
| `coral` | autos | `branch` (A to L) and `level` (L1 to L4): a coral scored there |
| `net` | `centre_h_algae.json` | none: an algae went into the net |
| `STOWED`, `PREPARING`, `OPENING`, `TENSIONING`, `LIFTING`, `PAYING_OUT`, `DEPLOYED`, `WINDING`, `CLIMBED` | `climb.json` | none: the climb entered this step |

The climb steps, in that order: stowed, waiting for the climb pose, the funnel opening, winding in to unload the pawls,
lifting them, letting the fork down, waiting for the cage, winding in, done. Between `DEPLOYED` and `WINDING` the driver
backs onto the cage, which the recording does not show: the robot stays where it is.

| Column | Meaning |
| --- | --- |
| `t_s` | Seconds since auto, or the climb, began |
| `x_m`, `y_m` | Robot centre on the field, metres from the blue corner: x away from the blue wall, y to the blue drivers' left |
| `heading_deg` | Way the front points: 0 is away from the blue wall, counter-clockwise positive. It wraps at 180 |
| `carriage_in` | Carriage height above rest, 0 to 66 |
| `wrist_deg` | -90 stowed, 0 level, positive tip up |
| `coral`, `algae` | 1 while one is really aboard |
| `fork_deg` | Climber fork: 90 stowed, 0 level and pointing back |
| `tray_deg` | Funnel tray: 0 closed, 90 open |

The driver cycle and vision runs add these. Values in quotes are listed in the file, under `phases` and `vision_states`,
and a row holds the number of one.

| Column | Meaning |
| --- | --- |
| `phase` | What the commands are doing: `idle`, `pickup`, `route`, `raise`, `release`, `stow` |
| `goal_carriage_in`, `goal_wrist_deg` | Where the planner sends each axis now |
| `target_x_m`, `target_y_m` | Where the drive is heading now |
| `est_x_m`, `est_y_m` | Where the robot thinks it is (`x_m`, `y_m` is where it really is) |
| `tag`, `tx_deg`, `tag_height_px` | The tag the camera sees (-1 for none), how far right of centre, how tall in pixels |
| `vision` | What was done with that sighting, such as `fix applied` or `fix too far from the estimate` |
| `fix_x_m`, `fix_y_m` | Where the sighting puts the robot, or null |

`tags` in those files lists every reef tag's place on the field.

## Moving the 3D model

`ascope/Robot_ClaudeBot/` has the CAD model as seven GLB files, in metres, Y up. In CAD axes (X forward, Y left,
Z up, inches, origin on the carpet at the robot's centre):

| File | Moves by |
| --- | --- |
| `model.glb` | Nothing: it is the robot itself |
| `model_0.glb` stage A | up `carriage / 3` |
| `model_1.glb` stage B | up `2 * carriage / 3` |
| `model_2.glb` carriage | up `carriage` |
| `model_3.glb` wrist | to (4.5, 0, 24.5 + carriage), turned `-wrist_deg` about +Y |
| `model_4.glb` fork | to (-13.5, 0, 16.0), turned `fork_deg - 90` about +Y |
| `model_5.glb` tray | to (0, -7.1, 27.5), turned `tray_deg` about +X |

The same in three.js. Put all seven in a group turned `rotation.x = Math.PI / 2`, which makes it Z up, or leave the
group alone and use Y up as here:

```js
const IN = 0.0254, RAD = Math.PI / 180;
function pose(parts, c, wrist, fork = 90, tray = 0) {           // parts[i] is the loaded model_i.glb
  parts[0].position.y = (c / 3) * IN;
  parts[1].position.y = ((2 * c) / 3) * IN;
  parts[2].position.y = c * IN;
  parts[3].position.set(4.5 * IN, (24.5 + c) * IN, 0);
  parts[3].rotation.z = wrist * RAD;
  parts[4].position.set(-13.5 * IN, 16 * IN, 0);
  parts[4].rotation.z = (90 - fork) * RAD;
  parts[5].position.set(0, 27.5 * IN, 7.1 * IN);
  parts[5].rotation.x = tray * RAD;
}
// The robot on the field, Y up: x stays x, field y becomes -z.
function place(robot, x, y, heading) {
  robot.position.set(x, 0, -y);
  robot.rotation.y = heading * RAD;
}
```

Parts are merged by colour, so this model cannot say which part was clicked. The kickoff brief's model names every
part; use that one for clicking, and this one for moving.
