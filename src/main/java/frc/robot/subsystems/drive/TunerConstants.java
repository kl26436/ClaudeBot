package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Ports;
import frc.robot.config.Ports.CanId;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.DriveK;

/**
 * Swerve constants, hand-written in the shape CTRE's Tuner X generator produces so a generated file
 * can be compared against it line by line. Module order is front-left, front-right, back-left,
 * back-right.
 */
public final class TunerConstants {
    private TunerConstants() {}

    // Steer: volts per rotation of the module.
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100) // PLACEHOLDER(tuning)
            .withKI(0)
            .withKD(0.5) // PLACEHOLDER(tuning)
            .withKS(0.1) // PLACEHOLDER(tuning)
            .withKV(2.66) // PLACEHOLDER(tuning)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // Drive: volts per rotor rotation per second.
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(0.1) // PLACEHOLDER(tuning)
            .withKI(0)
            .withKD(0)
            .withKS(0) // PLACEHOLDER(tuning)
            .withKV(0.124); // PLACEHOLDER(tuning)

    /** Drive stator limit. Raise it toward the slip current found on carpet with CTRE's slip test. */
    private static final double SLIP_CURRENT_A = DriveK.DRIVE_STATOR_LIMIT_A; // PLACEHOLDER(tuning)

    // MK4i: drive motors on the right side, and every steer motor, run inverted.
    private static final boolean LEFT_DRIVE_INVERTED = false;
    private static final boolean RIGHT_DRIVE_INVERTED = true;
    private static final boolean STEER_INVERTED = true;
    private static final boolean ENCODER_INVERTED = false;

    /** CANcoder offsets in rotations. Zero until the modules are aligned on the built robot. */
    private static final double FL_OFFSET_ROT = 0.0; // PLACEHOLDER(tuning)
    private static final double FR_OFFSET_ROT = 0.0; // PLACEHOLDER(tuning)
    private static final double BL_OFFSET_ROT = 0.0; // PLACEHOLDER(tuning)
    private static final double BR_OFFSET_ROT = 0.0; // PLACEHOLDER(tuning)

    // Simulated module inertias and friction (used only by the simulation).
    private static final double STEER_INERTIA_KGM2 = 0.01; // PLACEHOLDER(drawing)
    private static final double DRIVE_INERTIA_KGM2 = 0.01; // PLACEHOLDER(drawing)
    private static final double STEER_FRICTION_V = 0.2; // PLACEHOLDER(tuning)
    private static final double DRIVE_FRICTION_V = 0.2; // PLACEHOLDER(tuning)

    public static final double WHEEL_RADIUS_M = Units.inchesToMeters(DriveK.WHEEL_DIAMETER_IN / 2);
    private static final double MODULE_OFFSET_M = Units.inchesToMeters(DriveK.MODULE_OFFSET_IN);

    public static final SwerveDrivetrainConstants DRIVETRAIN = new SwerveDrivetrainConstants()
            .withCANBusName(Ports.DRIVE_BUS)
            .withPigeon2Id(Ports.PIGEON.id());

    private static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> factory() {
        var steerInitial = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(DriveK.STEER_STATOR_LIMIT_A)
                .withStatorCurrentLimitEnable(true));
        return new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DriveK.DRIVE_RATIO)
                .withSteerMotorGearRatio(DriveK.STEER_RATIO)
                .withCouplingGearRatio(DriveK.COUPLING_RATIO)
                .withWheelRadius(WHEEL_RADIUS_M)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(SLIP_CURRENT_A)
                .withSpeedAt12Volts(DriveK.MAX_SPEED_MPS)
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                // Fusing the CANcoder with the rotor needs Phoenix Pro; without it, read the CANcoder alone.
                .withFeedbackSource(
                        RobotConstants.PRO_LICENSED ? SteerFeedbackType.FusedCANcoder : SteerFeedbackType.RemoteCANcoder)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(steerInitial)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withSteerInertia(STEER_INERTIA_KGM2)
                .withDriveInertia(DRIVE_INERTIA_KGM2)
                .withSteerFrictionVoltage(STEER_FRICTION_V)
                .withDriveFrictionVoltage(DRIVE_FRICTION_V);
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> module(
            SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> f,
            CanId steer,
            CanId drive,
            CanId encoder,
            double offsetRot,
            double xM,
            double yM,
            boolean driveInverted) {
        return f.createModuleConstants(
                steer.id(), drive.id(), encoder.id(), offsetRot, xM, yM, driveInverted, STEER_INVERTED, ENCODER_INVERTED);
    }

    /**
     * Fresh module constants, front-left, front-right, back-left, back-right. A new array every call,
     * so the simulation can adjust its copy without touching the robot's.
     */
    @SuppressWarnings("unchecked")
    public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] modules() {
        var f = factory();
        double d = MODULE_OFFSET_M;
        return new SwerveModuleConstants[] {
            module(f, Ports.FL_STEER, Ports.FL_DRIVE, Ports.FL_ENCODER, FL_OFFSET_ROT, d, d, LEFT_DRIVE_INVERTED),
            module(f, Ports.FR_STEER, Ports.FR_DRIVE, Ports.FR_ENCODER, FR_OFFSET_ROT, d, -d, RIGHT_DRIVE_INVERTED),
            module(f, Ports.BL_STEER, Ports.BL_DRIVE, Ports.BL_ENCODER, BL_OFFSET_ROT, -d, d, LEFT_DRIVE_INVERTED),
            module(f, Ports.BR_STEER, Ports.BR_DRIVE, Ports.BR_ENCODER, BR_OFFSET_ROT, -d, -d, RIGHT_DRIVE_INVERTED),
        };
    }
}
