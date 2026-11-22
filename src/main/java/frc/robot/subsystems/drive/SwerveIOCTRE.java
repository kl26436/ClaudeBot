// Adapted from FRC Team 2910's 2026CompetitionRobot-Public (MIT licence; see NOTICE).
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * CTRE's swerve drivetrain on Phoenix 6 devices. CTRE runs odometry on its own thread; this class
 * copies the latest state out once per robot loop.
 */
public class SwerveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveIO {
    // Heading controller, radians to radians per second.
    private static final double HEADING_KP = 7.0; // PLACEHOLDER(tuning)
    private static final double HEADING_KD = 0.2; // PLACEHOLDER(tuning)

    private final FieldSpeedsRequest request = new FieldSpeedsRequest(HEADING_KP, HEADING_KD);
    private int lastSuccessfulDaqs = -1;

    @SafeVarargs
    public SwerveIOCTRE(
            SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, modules);
    }

    /** As above, with the odometry thread at {@code odometryHz} instead of CTRE's default. */
    @SafeVarargs
    protected SwerveIOCTRE(
            SwerveDrivetrainConstants constants,
            double odometryHz,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, odometryHz, modules);
    }

    /** The robot's drivetrain, with the constants from {@link TunerConstants}. */
    public SwerveIOCTRE() {
        this(TunerConstants.DRIVETRAIN, TunerConstants.modules());
    }

    @Override
    public void updateInputs(SwerveIOInputs in) {
        var state = getState();
        if (state.Pose != null) {
            in.pose = state.Pose;
        }
        if (state.Speeds != null) {
            in.speeds = state.Speeds;
        }
        if (state.ModuleStates != null) {
            in.moduleStates = state.ModuleStates.clone();
        }
        if (state.ModuleTargets != null) {
            in.moduleTargets = state.ModuleTargets.clone();
        }
        if (state.RawHeading != null) {
            in.gyroYaw = state.RawHeading;
        }
        // Odometry is alive while successful samples keep arriving between loops.
        in.connected = lastSuccessfulDaqs >= 0 && state.SuccessfulDaqs > lastSuccessfulDaqs;
        lastSuccessfulDaqs = state.SuccessfulDaqs;
    }

    @Override
    public void driveFieldRelative(double vxMps, double vyMps, double omegaRadPerS) {
        setControl(request.withSpeeds(vxMps, vyMps, omegaRadPerS));
    }

    @Override
    public void driveFieldRelativeFacing(double vxMps, double vyMps, Rotation2d heading, double maxOmegaRadPerS) {
        setControl(request.withHeading(vxMps, vyMps, heading, maxOmegaRadPerS));
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void resetTranslation(Translation2d t) {
        super.resetTranslation(t);
    }

    @Override
    public void stop() {
        setControl(request.withSpeeds(0, 0, 0));
    }

    /**
     * CTRE's close destroys the native drivetrain but leaves its devices open, still sending their
     * 250 Hz signals. Close them too, so a discarded drivetrain stops loading the bus.
     */
    @Override
    public void close() {
        super.close();
        for (var module : getModules()) {
            module.getDriveMotor().close();
            module.getSteerMotor().close();
            module.getEncoder().close();
        }
        getPigeon2().close();
    }
}
