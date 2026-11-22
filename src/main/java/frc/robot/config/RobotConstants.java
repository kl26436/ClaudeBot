package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

/** Robot-wide constants. Every dimension and ratio lives here. */
public final class RobotConstants {
    private RobotConstants() {}

    /** Run mode chosen at startup. */
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    /** What a non-real run does: {@link Mode#SIM} to simulate, {@link Mode#REPLAY} to replay a log. */
    private static final Mode SIM_MODE = Mode.SIM;

    public static final int TEAM_NUMBER = 4206;

    /** Phoenix Pro licence present. When false, no code path may request FOC, fused sensors or time-synced signals. */
    public static final boolean PRO_LICENSED = true;

    public static Mode currentMode() {
        return RobotBase.isReal() ? Mode.REAL : SIM_MODE;
    }

    /** AprilTag field layout used for vision and auto-align. */
    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;

    /** Swerve drivetrain. */
    public static final class DriveK {
        private DriveK() {}

        public static final double MODULE_OFFSET_IN = 11.373;
        public static final double WHEEL_DIAMETER_IN = 3.947;
        public static final double DRIVE_RATIO = 6.75;
        public static final double STEER_RATIO = 150.0 / 7.0;
        public static final double BUMPER_FACE_FROM_CENTRE_IN = 17.4; // PLACEHOLDER(drawing)
        public static final double MAX_SPEED_MPS = 4.67;
        public static final double ROBOT_MASS_LB = 140; // PLACEHOLDER(drawing)
    }

    /** Elevator carriage. Units are inches of carriage travel. */
    public static final class ElevatorK {
        private ElevatorK() {}

        public static final double SPROCKET_TEETH = 22;
        public static final double CHAIN_PITCH_IN = 0.25;
        public static final double GEAR_RATIO = 12;
        public static final double STAGES = 3;
        /** Carriage inches per motor rotation: sprocket circumference per output turn, times stages, over the gearbox. */
        public static final double IN_PER_MOTOR_ROT = SPROCKET_TEETH * CHAIN_PITCH_IN * STAGES / GEAR_RATIO;
        public static final double TRAVEL_IN = 66;
        public static final double SOFT_MAX_IN = 65.25;
        public static final double MAX_ACCEL_IN_PER_S2 = 0.6 * 386.09;
        public static final double CRUISE_IN_PER_S = 65; // PLACEHOLDER(tuning)
        public static final double PIVOT_REST_HEIGHT_IN = 24.5;
        public static final double PIVOT_X_IN = 4.5;
        public static final double CARRIAGE_GROUP_LB = 25;
        public static final double STAGE_A_LB = 5; // PLACEHOLDER(drawing)
        public static final double STAGE_B_LB = 4; // PLACEHOLDER(drawing)
        public static final double HOME_VOLTS = -1.5;
        public static final double HOME_TIMEOUT_S = 3.0;
    }

    /** Wrist. Units are degrees from level, positive up. */
    public static final class WristK {
        private WristK() {}

        public static final double GEAR_RATIO = 60;
        public static final double MIN_DEG = -90;
        public static final double MAX_DEG = 75;
        public static final double HARD_MIN_DEG = -93;
        public static final double HARD_MAX_DEG = 77;
        public static final double STATOR_LIMIT_A = 40;
        public static final double TURNING_LB = 15.3;
        public static final double GRAVITY_LBIN_EMPTY = 125;
        public static final double GRAVITY_LBIN_CORAL = 138;
        public static final double GRAVITY_LBIN_ALGAE = 151;
        public static final double MOI_KGM2 = 0.39; // PLACEHOLDER(drawing)
        public static final double CRUISE_DEG_PER_S = 400; // PLACEHOLDER(tuning)
        public static final double ACCEL_DEG_PER_S2 = 1600; // PLACEHOLDER(tuning)
    }

    /** Coral roller. */
    public static final class CoralK {
        private CoralK() {}

        public static final double GEAR_RATIO = 3;
        public static final double WHEEL_DIAMETER_IN = 2;
        public static final double STATOR_LIMIT_A = 40; // PLACEHOLDER(tuning)
        /** Roller inertia at the output shaft, for the simulation. */
        public static final double MOI_KGM2 = 0.0015; // PLACEHOLDER(drawing)
    }

    /** Algae claw. */
    public static final class ClawK {
        private ClawK() {}

        public static final double GEAR_RATIO = 2;
        public static final double STATOR_LIMIT_A = 30;
        /** Inertia of one claw roller at its output shaft, for the simulation. */
        public static final double MOI_KGM2 = 0.003; // PLACEHOLDER(drawing)
    }

    /** Intake camera. */
    public static final class CameraK {
        private CameraK() {}

        public static final double HEIGHT_IN = 9.57;
        public static final double PITCH_UP_DEG = 12;
        public static final double FORWARD_IN = 12.0; // PLACEHOLDER(drawing)
        public static final double H_FOV_DEG = 82;
        public static final double V_FOV_DEG = 56.2;
        public static final double H_PIXELS = 1280;
        public static final double V_PIXELS = 800;
    }
}
