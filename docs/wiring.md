# Robot A wiring map

Copied verbatim from the CAN IDs and Wiring sections of the design ([Robot A code design](https://claude.ai/artifact/6YYpCCkQaGtHnapUWu4cRd), revision 36, approved 2026-11-21). Every ID here is declared once, in `src/main/java/frc/robot/config/Ports.java`; `PortsTest` checks the two agree on count and uniqueness. At start-up the robot calls every device in `Ports.all()` and raises a dashboard alert naming any that do not answer.

## CAN IDs

The robot has two CAN buses with 23 devices between them, and no ID number is used twice.

- **`drive` bus, on a CANivore:** the four swerve modules and the gyro, 13 devices.
- **`rio` bus, on the roboRIO:** power distribution and everything on the elevator and wrist, 10 devices.

The split isolates faults. The harness that rides the elevator is the wire most likely to fail, and it should not take the drivetrain down with it.

| ID | Device | Bus | Where it sits |
| --- | --- | --- | --- |
| 1 | REV Power Distribution Hub | rio | Belly pan |
| 2 | Pigeon 2 gyro | drive | Belly pan, near robot centre |
| 11, 12, 13 | Front-left drive, steer, CANcoder | drive | Front-left module |
| 21, 22, 23 | Front-right drive, steer, CANcoder | drive | Front-right module |
| 31, 32, 33 | Back-left drive, steer, CANcoder | drive | Back-left module |
| 41, 42, 43 | Back-right drive, steer, CANcoder | drive | Back-right module |
| 50 | Elevator left motor, the leader | rio | Lift gearbox |
| 51 | Elevator right motor, the follower | rio | Lift gearbox |
| 52 | Wrist motor | rio | Carriage |
| 53 | CANdi reading the wrist encoder | rio | Carriage, at the pivot |
| 55 | Coral roller motor | rio | Wrist |
| 56 | Claw front roller motor | rio | Wrist |
| 57 | Claw rear roller motor | rio | Wrist |
| 58 | Coral CANrange | rio | Wrist |
| 59 | Algae CANrange | rio | Wrist |
| 60, 61, 62 | Reserved for the climber (60, 61) and LEDs (62) |  |  |

The numbers read by location. A tens digit of 1 to 4 is a swerve corner, 50 to 53 ride the mast and carriage, and 55 to 59 are on the wrist. CTRE devices accept IDs up to 62.

**Chain order on the `drive` bus**

1. CANivore, plugged into the roboRIO by USB. Its built-in terminator ends the bus.
2. Front-left module: 13, 12, 11
3. Front-right module: 23, 22, 21
4. Back-right module: 43, 42, 41
5. Back-left module: 33, 32, 31
6. Pigeon 2
7. A 120 Ω terminator

**Chain order on the `rio` bus**

1. roboRIO, which has a built-in terminator
2. Power Distribution Hub, with its terminator switch off
3. Elevator left motor, 50
4. Elevator right motor, 51
5. Up the cable chain to the carriage
6. Wrist motor, 52
7. CANdi, 53
8. Over the wiring loop at the wrist pivot
9. Coral roller motor, 55
10. Claw front and rear motors, 56 and 57
11. Coral and algae CANrange, 58 and 59
12. A 120 Ω terminator on the wrist

One CAN pair goes up the elevator and none comes back. The order along each chain can change to suit the electronics layout without touching the code.

## Wiring

Power comes from one REV Power Distribution Hub, with breaker and wire sizes taken from rules R615 to R622 of the 2025 manual in this project. The electronics layout is not designed yet, so channel numbers can move. The code uses them only to log current.

**Power channels**

| Hub channel | Protection | Smallest wire | Load |
| --- | --- | --- | --- |
| 0 to 7 | 40 A breaker each | 12 AWG | The eight swerve motors, one per channel |
| 8, 9 | 40 A breaker each | 12 AWG | Elevator left and right motors |
| 10 | 30 A breaker | 14 AWG | Wrist motor |
| 11 | 30 A breaker | 14 AWG | Coral roller motor |
| 12, 13 | 30 A breaker each | 14 AWG | Claw front and rear motors |
| 14 | 20 A breaker | 18 AWG | REV Mini Power Module for the CAN sensors |
| 15 to 19 |  |  | Spare. The climber takes two. |
| 20 | 10 A fuse | 22 AWG | roboRIO (R615) |
| 21 | 10 A fuse | 18 AWG | Radio, alone on its channel (R617) |
| 22 | 5 A fuse | 22 AWG | Limelight 4 |
| 23, switched |  |  | Spare, for LEDs |

The Mini Power Module has three fused outputs in use, each with 22 AWG wire and a fuse of 5 A or less:

1. CANivore
2. Pigeon 2 and the four CANcoders
3. Carriage sensors: the CANdi and both CANrange

**Signals**

| Signal | Device | Lands on |
| --- | --- | --- |
| Elevator home | Magnetic switch on post E-109 | roboRIO DIO 0 |
| Wrist angle | REV Through Bore Encoder, absolute output | CANdi input 1, on the carriage |
| Coral and algae present | Two CANrange | `rio` CAN bus |
| Camera | Limelight 4 | Ethernet to the radio's AUX 1 port |
| Robot network | roboRIO | Ethernet to the radio's RIO port (R703) |
| Drive bus | CANivore | roboRIO USB |

**What rides the moving harness**

- **Up the cable chain to the carriage, 12 conductors:** four motor pairs in 14 AWG (wrist, coral, two claw), one CAN pair and one 22 AWG sensor power pair.
- **Over the wrist pivot loop, 10 conductors:** three motor pairs, the CAN pair and the sensor power pair.

The wrist and roller motors are on 30 A breakers so their wire up the chain can be 14 AWG instead of 12. The fuse sizes for the Limelight and sensors are my proposal, so check each device's guide when wiring.

