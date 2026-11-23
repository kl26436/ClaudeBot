package frc.robot.config;

import java.util.List;

/**
 * Every CAN ID, bus name and roboRIO port in the robot. Nothing else declares a device address.
 *
 * <p>Two CAN buses: {@link #DRIVE_BUS} carries the swerve modules and the Pigeon 2; {@link
 * #RIO_BUS} carries the power hub, the mechanisms and their sensors. The values are the CAD package's
 * CAN map, {@code docs/cad/robotA_can_map.csv}; a test compares the two.
 */
public final class Ports {
    private Ports() {}

    /** A device address: a CAN ID on a named bus. */
    public record CanId(int id, String bus) {}

    /** A device with a readable name and its CTRE or REV device kind. */
    public record Device(String name, CanId can, String kind) {}

    public static final String DRIVE_BUS = "canivore";
    public static final String RIO_BUS = "rio";

    public static final CanId PDH = new CanId(1, RIO_BUS);
    public static final CanId PIGEON = new CanId(10, DRIVE_BUS);

    // Swerve modules on the drive bus: drive motor, steer motor, steer encoder.
    public static final CanId FL_DRIVE = new CanId(11, DRIVE_BUS);
    public static final CanId FL_STEER = new CanId(12, DRIVE_BUS);
    public static final CanId FL_ENCODER = new CanId(13, DRIVE_BUS);
    public static final CanId FR_DRIVE = new CanId(21, DRIVE_BUS);
    public static final CanId FR_STEER = new CanId(22, DRIVE_BUS);
    public static final CanId FR_ENCODER = new CanId(23, DRIVE_BUS);
    public static final CanId BL_DRIVE = new CanId(31, DRIVE_BUS);
    public static final CanId BL_STEER = new CanId(32, DRIVE_BUS);
    public static final CanId BL_ENCODER = new CanId(33, DRIVE_BUS);
    public static final CanId BR_DRIVE = new CanId(41, DRIVE_BUS);
    public static final CanId BR_STEER = new CanId(42, DRIVE_BUS);
    public static final CanId BR_ENCODER = new CanId(43, DRIVE_BUS);

    // Mechanisms on the rio bus.
    public static final CanId ELEVATOR_LEFT = new CanId(51, RIO_BUS);
    public static final CanId ELEVATOR_RIGHT = new CanId(52, RIO_BUS);
    public static final CanId WRIST = new CanId(53, RIO_BUS);
    public static final CanId CORAL_ROLLERS = new CanId(54, RIO_BUS);
    public static final CanId CLAW_FRONT = new CanId(55, RIO_BUS);
    public static final CanId CLAW_REAR = new CanId(56, RIO_BUS);
    public static final CanId CLIMBER_LEFT = new CanId(57, RIO_BUS);
    public static final CanId CLIMBER_RIGHT = new CanId(58, RIO_BUS);
    public static final CanId CORAL_RANGE = new CanId(61, RIO_BUS);
    public static final CanId ALGAE_RANGE = new CanId(62, RIO_BUS);

    /** roboRIO digital input for the elevator home switch. */
    public static final int ELEVATOR_HOME_DIO = 0;

    /** roboRIO digital input for the wrist's absolute encoder (duty cycle). */
    public static final int WRIST_ENCODER_DIO = 1;

    /** roboRIO PWM output for the funnel latch servo. */
    public static final int FUNNEL_LATCH_PWM = 0;

    /** roboRIO PWM output for both climber pawl servos, on one Y lead. */
    public static final int CLIMBER_PAWL_PWM = 1;

    /** Every device on the robot, in one list, for the ID-uniqueness and count checks. */
    public static List<Device> all() {
        return List.of(
                new Device("Power distribution hub", PDH, "PDH"),
                new Device("Pigeon 2 gyro", PIGEON, "Pigeon2"),
                new Device("Front-left drive motor", FL_DRIVE, "TalonFX"),
                new Device("Front-left steer motor", FL_STEER, "TalonFX"),
                new Device("Front-left steer encoder", FL_ENCODER, "CANcoder"),
                new Device("Front-right drive motor", FR_DRIVE, "TalonFX"),
                new Device("Front-right steer motor", FR_STEER, "TalonFX"),
                new Device("Front-right steer encoder", FR_ENCODER, "CANcoder"),
                new Device("Back-left drive motor", BL_DRIVE, "TalonFX"),
                new Device("Back-left steer motor", BL_STEER, "TalonFX"),
                new Device("Back-left steer encoder", BL_ENCODER, "CANcoder"),
                new Device("Back-right drive motor", BR_DRIVE, "TalonFX"),
                new Device("Back-right steer motor", BR_STEER, "TalonFX"),
                new Device("Back-right steer encoder", BR_ENCODER, "CANcoder"),
                new Device("Elevator left motor", ELEVATOR_LEFT, "TalonFX"),
                new Device("Elevator right motor", ELEVATOR_RIGHT, "TalonFX"),
                new Device("Wrist motor", WRIST, "TalonFX"),
                new Device("Coral rollers motor", CORAL_ROLLERS, "TalonFX"),
                new Device("Front claw motor", CLAW_FRONT, "TalonFX"),
                new Device("Rear claw motor", CLAW_REAR, "TalonFX"),
                new Device("Climber left motor", CLIMBER_LEFT, "TalonFX"),
                new Device("Climber right motor", CLIMBER_RIGHT, "TalonFX"),
                new Device("Coral range sensor", CORAL_RANGE, "CANrange"),
                new Device("Algae range sensor", ALGAE_RANGE, "CANrange"));
    }
}
