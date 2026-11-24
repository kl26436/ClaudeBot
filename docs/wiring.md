# ClaudeBot wiring map

From the CAD package rev3 (2026-11-23), `docs/cad/can_map.csv`; step-by-step wiring is in the
[ClaudeBot build instructions](https://claude.ai/artifact/NBBpmCtf4Wyv4jgVFkgQrD), section 8. Every ID is
declared once, in `src/main/java/frc/robot/config/Ports.java`, and `PortsTest` compares that file with the
CAD list device by device. At start-up the robot calls every device in `Ports.all()` and raises a dashboard
alert naming any that do not answer.

This replaces the CAN and power tables in the November 21 design.

## CAN devices and power channels

The CANivore has to be named `canivore` in Phoenix Tuner X. Set every ID before the device goes on the robot.

| ID | Bus | Device | Function | Power channel | Protection |
| --- | --- | --- | --- | --- | --- |
| 1 | rio | REV PDH | power distribution | - | - |
| 10 | canivore | Pigeon 2.0 | gyro | MPM 1 | 5 A fuse |
| 11 | canivore | Kraken X60 | front left drive | 18 | 40 A |
| 12 | canivore | Kraken X60 | front left steer | 19 | 40 A |
| 13 | canivore | CANcoder | front left angle | MPM 2 | 5 A fuse |
| 21 | canivore | Kraken X60 | front right drive | 1 | 40 A |
| 22 | canivore | Kraken X60 | front right steer | 0 | 40 A |
| 23 | canivore | CANcoder | front right angle | MPM 2 | 5 A fuse |
| 31 | canivore | Kraken X60 | back left drive | 11 | 40 A |
| 32 | canivore | Kraken X60 | back left steer | 10 | 40 A |
| 33 | canivore | CANcoder | back left angle | MPM 2 | 5 A fuse |
| 41 | canivore | Kraken X60 | back right drive | 8 | 40 A |
| 42 | canivore | Kraken X60 | back right steer | 9 | 40 A |
| 43 | canivore | CANcoder | back right angle | MPM 2 | 5 A fuse |
| 51 | rio | Kraken X60 | elevator lift, left motor | 16 | 40 A |
| 52 | rio | Kraken X60 | elevator lift, right motor | 3 | 40 A |
| 53 | rio | Kraken X60 | wrist | 15 | 40 A |
| 54 | rio | Kraken X44 | coral rollers | 14 | 40 A |
| 55 | rio | Kraken X44 | claw front roller | 13 | 40 A |
| 56 | rio | Kraken X44 | claw rear roller | 12 | 40 A |
| 57 | rio | Kraken X60 | climber, left motor | 17 | 40 A |
| 58 | rio | Kraken X60 | climber, right motor | 2 | 40 A |
| 61 | rio | CANrange | coral in the wrist | MPM 3 | 5 A fuse |
| 62 | rio | CANrange | algae in the claw | MPM 3 | 5 A fuse |

## Other power

| Other power | Load | Protection |
| --- | --- | --- |
| PDH 4 | Limelight 4 | 10 A breaker |
| PDH 5 | Mini Power Module feed | 20 A breaker |
| PDH 6, 7 | spare | - |
| PDH 20 | roboRIO | 10 A fuse |
| PDH 21 | radio VH-109, 12 V direct (no PoE) | 10 A fuse |
| PDH 22 | spare | - |
| PDH 23 (switched) | spare (lights) | - |
| MPM 4 to 6 | spare | - |

## roboRIO ports

| RoboRIO port | Use |
| --- | --- |
| DIO 0 | elevator home switch (REV magnetic switch on post E-109) |
| DIO 1 | wrist absolute encoder (REV through bore, duty cycle) |
| PWM 0 | funnel latch servo (signal and 6 V from the roboRIO) |
| PWM 1 | climber pawl servos, both on one Y lead (lift the pawls to pay the rope out) |
| USB | CANivore |
| Ethernet | radio port RIO; Limelight 4 on radio port AUX 1 |

The three servos run from the roboRIO's 6 V rail. If it browns out when they stall, move them to a REV Servo
Power Module on a spare PDH channel (sheet EL-001).

## Order along each bus

| Bus | Order of devices | Note |
| --- | --- | --- |
| canivore (CAN FD) | CANivore > Pigeon 2 > back left module > back right module > front right module > front left module > 120 ohm terminator | drivetrain only: 8 motors, 4 CANcoders, gyro |
| rio | roboRIO > PDH (in and out, terminator switch OFF) > climber right > climber left > lift right > lift left > up the cable chain > wrist > coral rollers > coral CANrange > claw front > claw rear > algae CANrange > 120 ohm terminator | mechanisms and the PDH; the roboRIO ends the bus at one end, the resistor at the claw ends the other |

## Starting current limits (sheet EL-001)

Drive 60 A stator, steer 30, lift 60, wrist 40, rollers 30, climber 60. Every Kraken channel has a 40 A breaker.
