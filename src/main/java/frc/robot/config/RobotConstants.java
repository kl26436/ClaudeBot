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
        /** Drive motor turns per steer turn with the wheel held (SDS MK4i: 50/14). */
        public static final double COUPLING_RATIO = 50.0 / 14.0; // PLACEHOLDER(drawing)
        public static final double BUMPER_FACE_FROM_CENTRE_IN = 34.5 / 2;
        public static final double MAX_SPEED_MPS = 4.67;
        /** On the field with battery and bumpers (drawing R-001, rev3; estimated, not weighed). */
        public static final double ROBOT_MASS_LB = 142.8;
        /** Starting current limits from drawing EL-001. */
        public static final double DRIVE_STATOR_LIMIT_A = 60;

        public static final double STEER_STATOR_LIMIT_A = 30;

        /**
         * Steady acceleration that lifts the far wheels, in g, by carriage height (inches): the lowest of
         * forward, back and sideways on drawing R-001 (rev3 balance table, battery and bumpers on).
         */
        public static final double[] TIP_CARRIAGE_IN = {0, 33, 66};

        public static final double[] TIP_ACCEL_G = {1.07, 0.71, 0.50};
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
        /** Nothing hauls the carriage down, so it may not be lowered faster than this (elevator Rev F). */
        public static final double MAX_ACCEL_IN_PER_S2 = 0.6 * 386.09;
        /** What the motion profile asks for: under the cap, leaving room for the carriage to catch up with it. */
        public static final double ACCEL_IN_PER_S2 = 0.55 * 386.09;

        public static final double CRUISE_IN_PER_S = 110; // PLACEHOLDER(tuning)
        public static final double PIVOT_REST_HEIGHT_IN = 24.5;
        public static final double PIVOT_X_IN = 4.5;
        public static final double CARRIAGE_GROUP_LB = 25;
        public static final double STAGE_A_LB = 5; // PLACEHOLDER(drawing)
        public static final double STAGE_B_LB = 4; // PLACEHOLDER(drawing)
        public static final double STATOR_LIMIT_A = 60;
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
        public static final double TURNING_LB = 13.3;
        public static final double GRAVITY_LBIN_EMPTY = 110;
        public static final double GRAVITY_LBIN_CORAL = 124;
        public static final double GRAVITY_LBIN_ALGAE = 137;
        public static final double MOI_KGM2 = 0.39; // PLACEHOLDER(drawing)
        public static final double CRUISE_DEG_PER_S = 400; // PLACEHOLDER(tuning)
        public static final double ACCEL_DEG_PER_S2 = 1600; // PLACEHOLDER(tuning)
    }

    /** Coral roller. */
    public static final class CoralK {
        private CoralK() {}

        public static final double GEAR_RATIO = 3;
        public static final double WHEEL_DIAMETER_IN = 2;
        public static final double STATOR_LIMIT_A = 30;
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

    /** Climber winch, pawls and funnel latch. Fork angles: 90 stowed, 0 level. */
    public static final class ClimberK {
        private ClimberK() {}

        /** Motor turns per fork turn: 25:1 gearbox, then the spool on a 3.5 in rope track. */
        public static final double GEAR_RATIO = 25 * 3.5 / 0.56;

        public static final double STOWED_DEG = 90;
        public static final double LEVEL_DEG = 0;
        /** Stop bolts; the fork rests on the first with the rope slack. */
        public static final double LEVEL_STOP_DEG = -2;

        public static final double HARD_MAX_DEG = 97;
        /** Where the climb stops, short of the hard stop. */
        public static final double CLIMBED_DEG = 88;

        public static final double CLIMBED_TOL_DEG = 1.5;
        public static final double LEVEL_TOL_DEG = 1.0;
        /** One ratchet tooth of fork travel. */
        public static final double TOOTH_DEG = 3.6;

        /** Wound in this far to take the load off the pawls before lifting them. */
        public static final double TENSION_DEG = 3; // PLACEHOLDER(tuning)

        public static final double TENSION_MAX_DEG = 95;
        /** Winding in the empty fork takes a few amps; this much for this long is the stop bolts or the robot. */
        public static final double TENSION_LOAD_A = 20; // PLACEHOLDER(tuning)

        public static final double TENSION_LOAD_S = 0.1;
        /** Held still under that current for this long, the rope is taken to be unloaded and the pawls may lift. */
        public static final double TENSION_QUIET_S = 0.3;

        /** The button is held at least this long before the funnel opens, and before a wind-in starts. */
        public static final double COMMIT_HOLD_S = 0.7;

        public static final double CLIMB_HOLD_S = 0.3;
        public static final double TENSION_TOL_DEG = 1.0;
        public static final double STATOR_LIMIT_A = 60;
        public static final double CRUISE_DEG_PER_S = 60; // PLACEHOLDER(tuning)
        public static final double ACCEL_DEG_PER_S2 = 240; // PLACEHOLDER(tuning)
        /** Worst torque on the fork while lifting the robot (drawing CL-000). */
        public static final double CLIMB_LBIN = 2140;

        public static final double STALL_A = 55; // PLACEHOLDER(tuning)
        public static final double STALL_DEG_PER_S = 2;
        public static final double STALL_S = 0.5;

        /** Servo horn swings: pawls 145 to 123 degrees, latch 10 up to 32 down. */
        public static final double PAWL_HORN_SWING_DEG = 22;

        public static final double LATCH_HORN_SWING_DEG = 42;
        /** Time for the tubing to throw the tray open; nothing senses it. */
        public static final double FUNNEL_OPEN_S = 1.0; // PLACEHOLDER(robot-check)
        /** Time for the servos to lift the pawls; nothing senses them. */
        public static final double PAWL_LIFT_S = 0.4; // PLACEHOLDER(robot-check)

        // Joint positions in the robot frame, inches, for AdvantageScope.
        public static final double PIVOT_X_IN = -13.5;
        public static final double PIVOT_Z_IN = 16.0;
        public static final double TRAY_HINGE_Y_IN = -7.1;
        public static final double TRAY_HINGE_Z_IN = 27.5;
    }

    /** Intake camera. */
    public static final class CameraK {
        private CameraK() {}

        // Limelight 4 body centre (13.0, 0, 9.45), pitched up 12 degrees, lens 0.64 in ahead of it
        // along the optical axis (drivebase Rev F).
        public static final double PITCH_UP_DEG = 12;
        public static final double HEIGHT_IN = 9.45 + 0.64 * Math.sin(Math.toRadians(PITCH_UP_DEG));
        public static final double FORWARD_IN = 13.0 + 0.64 * Math.cos(Math.toRadians(PITCH_UP_DEG));
        public static final double H_FOV_DEG = 82;
        public static final double V_FOV_DEG = 56.2;
        public static final double H_PIXELS = 1280;
        public static final double V_PIXELS = 800;
    }
}
