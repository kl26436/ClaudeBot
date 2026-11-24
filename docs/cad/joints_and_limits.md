# ClaudeBot: joints and limits for the Onshape assemblies

Every joint that moves, with the limits that make the stops work. Zero = the pose each package is uploaded in (climber stowed, funnel closed, carriage down, wrist stowed). As built in Onshape (read back 2026-11-24): the direction is the first mate connector's Z, and Min and Max are the values the Onshape mate shows. Onshape counts a mate the opposite way to the right-hand rule about that Z (checked by moving a climber pawl), so the notes say in words which end is which. The Stops column is in the same order as Min / Max.

The code does not use these mate values. It works in its own conventions: carriage inches above rest, wrist theta (theta = -(mate + 90)), fork psi (psi = mate + 90), tray 0 closed to 90 open (mate 0 to -90). The motions agree with this list.

**Needed: 12 joints.** The optional ones only make rollers, wheels and servo links animate.

## Climber

| Joint | Type | Axis: point, direction | Min | Max | Stops | |
|---|---|---|---|---|---|---|
| Fork pivot (both forks on the one axle) | revolute | (-13.50, 0.00, 16.00), -Y | -92 deg | 7 deg | fork level (psi -2): stop bolt S0 + spacer CL-308 on each tower plate; the rope sets 0 in a climb / psi 97: stop bolt S97 + spacer CL-308 (the climb itself ends at 88 in code) | needed |
| Pawl, left | revolute | (-10.44, 5.06, 7.90), -Y | 0 deg | 16 deg | down in the WCP-0405 ratchet wheel (spring holds it there) / lifted 16 deg by the servo (clears the teeth by 0.10) | needed |
| Gate finger, left | revolute | (-6.20, 4.44, 27.45), -X | 0 deg | 90 deg | closed: across the fork opening, against the +rho end of the arm plate slot (it cannot open outward) / folded 90 back through the slot into block CL-205 (tip towards the axle) | needed |
| Pawl release linkage, left | revolute x3 (closed loop) | (-10.16, 5.52, 9.80), +Y | free |  | follows the pawl / follows the pawl | optional |
| Pawl, right | revolute | (-10.44, -5.06, 7.90), +Y | -16 deg | 0 deg | lifted 16 deg by the servo (clears the teeth by 0.10) / down in the WCP-0405 ratchet wheel (spring holds it there) | needed |
| Gate finger, right | revolute | (-6.20, -4.44, 27.45), -X | -90 deg | 0 deg | folded 90 back through the slot into block CL-205 (tip towards the axle) / closed: across the fork opening, against the +rho end of the arm plate slot (it cannot open outward) | needed |
| Pawl release linkage, right | revolute x3 (closed loop) | (-10.16, -5.52, 9.80), +Y | free |  | follows the pawl / follows the pawl | optional |
| Spools, ratchet wheels, gearbox outputs | revolute | (-9.44, 0.00, 7.90), +Y | free |  | none (winds both ways; the pawl holds it in the match) / none | optional |

- Fork pivot (both forks on the one axle): uploaded stowed (psi 90). Onshape: 0 = stowed, -92 = fork level on the S0 stops, +7 = on the 97 deg stops. Moves: fork group: CL-201 arm plates, fork tie rods, gate blocks CL-205 and fingers CL-203, entry guides CL-206, spring hubs CL-208, bronze bearings. Against: towers CL-101, the hex axle (it does not turn: the hex holes in the towers lock it), spools, gearboxes, frame.
- Pawl, left: turns on the rear gearbox spacer CL-306. Onshape: 0 = in the ratchet, +16 = lifted by the servo. Moves: WCP-0405 pawl (climber left). Against: tower CL-101-L (turns on the rear gearbox spacer CL-306).
- Gate finger, left: axis = the vertical #10 screw at psi 0 (along X with the fork stowed). Onshape: 0 = closed on the slot end, +90 = folded into CL-205. Moves: CL-203 gate finger (left fork). Against: gate block CL-205 / arm plate CL-201 (moves with the fork).
- Pawl release linkage, left: horn about the servo shaft, link on the horn pin, link on the pawl tail pin: the pawl limit then drives the servo horn. Moves: servo horn, CL-311 link. Against: servo case on the tower; link pins at the horn [-10.646, 9.928] and the pawl tail [-11.19, 7.9] (X, Z).
- Pawl, right: turns on the rear gearbox spacer CL-306. Onshape: 0 = in the ratchet, -16 = lifted by the servo. Moves: WCP-0405 pawl (climber right). Against: tower CL-101-R (turns on the rear gearbox spacer CL-306).
- Gate finger, right: axis = the vertical #10 screw at psi 0 (along X with the fork stowed). Onshape: 0 = closed on the slot end, -90 = folded into CL-205. Moves: CL-203 gate finger (right fork). Against: gate block CL-205 / arm plate CL-201 (moves with the fork).
- Pawl release linkage, right: horn about the servo shaft, link on the horn pin, link on the pawl tail pin: the pawl limit then drives the servo horn. Moves: servo horn, CL-311 link. Against: servo case on the tower; link pins at the horn [-10.646, 9.928] and the pawl tail [-11.19, 7.9] (X, Z).
- Spools, ratchet wheels, gearbox outputs: no limits: leave fastened unless you want to animate the winch. Moves: spool + WCP-0405 wheel each side. Against: axle / gearbox.

## Funnel

| Joint | Type | Axis: point, direction | Min | Max | Stops | |
|---|---|---|---|---|---|---|
| Tray hinge | revolute | (-4.00, -7.10, 27.50), +X | -90 deg | 0 deg | open 90 deg: drop link F-112 lands on the open stop screw head in the rear pivot plate F-111 (latex tubing throws it open) / closed: tray rests on the left posts (latch hook F-113 holds it) | needed |
| Latch hook | revolute | (-4.00, 7.10, 26.00), +Y | -30 deg | 0 deg | open 30 deg (26.5 clears the tube), servo at full swing / closed: finger over the stub end of the mid cross tube (spring) | needed |
| Latch release linkage | revolute x3 (closed loop) | (-4.00, 7.10, 23.62), +Y | free |  | follows the hook / follows the hook | optional |

- Tray hinge: one revolute on the mid pivot bolt only: the rear pivot bolt (F-111 / F-112) is on the same axis, a second mate would over-constrain it. Onshape: 0 = closed on the left posts, -90 = open. Moves: tray group: F-201 tray, F-202 walls, F-203 angles, F-102 / F-104 cross tubes, F-107 / F-108 saddles, F-112 drop link. Against: posts F-101 / F-103, pivot plates F-110 / F-111, F-105 gussets, frame.
- Latch hook: Onshape: 0 = closed over the cross tube stub, -30 = open. Moves: F-113 latch hook. Against: left mid post F-103-L (#10-32 bolt).
- Latch release linkage: same as the climber pawl linkage. Moves: servo horn, CL-307 link. Against: servo case on the post.

## Elevator

| Joint | Type | Axis: point, direction | Min | Max | Stops | |
|---|---|---|---|---|---|---|
| Stage A slide | slider | (4.50, 0.00, 0.00), +Z | -22 in | 0 in | up 22 (cascade: 3 x 22 = 66 at the carriage) / down on the bottom stops | needed |
| Stage B slide | slider | (4.50, 0.00, 0.00), -Z | 0 in | 22 in | down on the bottom stops / up 22 (cascade: 3 x 22 = 66 at the carriage) | needed |
| Carriage slide | slider | (4.50, 0.00, 0.00), +Z | -22 in | 0 in | up 22 (cascade: 3 x 22 = 66 at the carriage) / down on the bottom stops | needed |
| Cascade: Stage B = Stage A, carriage = Stage A | linear relation x2 | (4.50, 0.00, 0.00), +Z | ratio 1 |  | continuous rigging: every member moves the same relative amount /  | needed |

- Stage A slide: Onshape: 0 = down, -22 = up. Moves: Stage A. Against: mast.
- Stage B slide: Onshape: 0 = down, +22 = up. Moves: Stage B. Against: Stage A.
- Carriage slide: Onshape: 0 = down, -22 = up. Moves: carriage (with the wrist pivot plates E-403 and hubs). Against: Stage B.
- Cascade: Stage B = Stage A, carriage = Stage A: two linear relations, ratio 1: drag the carriage and the stages follow. Moves: the three slides. Against: -.

## Wrist

| Joint | Type | Axis: point, direction | Min | Max | Stops | |
|---|---|---|---|---|---|---|
| Wrist pivot | revolute | (4.50, 0.00, 24.50), -Y | -167 deg | 3 deg | theta +77: the other lug on the post / theta -93: lug W-210 on post W-301 | needed |
| Coral wheel shafts, claw rollers | revolute | (0.00, 0.00, 0.00), +Y | free |  | none / none | optional |

- Wrist pivot: uploaded stowed (theta -90). Onshape: 0 = stowed, -167 = far stop (tip up, theta +77), +3 = lug W-210 on the post (theta -93). Code rule the mate cannot hold: the wrist turns only with the carriage at 1.25 or higher. Moves: wrist and claw group (everything in wrist.py rot, the stub axles, W-103, collars, sprocket W-104). Against: carriage: pivot plates E-403, hubs W-101, encoder plate W-302, stop post W-301.
- Coral wheel shafts, claw rollers: no limits: leave fastened unless you want them to turn. Moves: shafts with wheels and pulleys. Against: wrist plates.

## Drivebase

| Joint | Type | Axis: point, direction | Min | Max | Stops | |
|---|---|---|---|---|---|---|
| MK4i steer and wheel, each module | revolute x2 | (0.00, 0.00, 0.00), +Z | free |  | none / none | optional |

- MK4i steer and wheel, each module: no limits; needs each module split into groups, so leave the modules fixed for now. Moves: module steer group; wheel. Against: module frame.
