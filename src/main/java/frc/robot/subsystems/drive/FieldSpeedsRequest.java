package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.RobotConstants;

/**
 * Field-relative swerve request, optionally turning to a target heading. CTRE's own field-centric
 * requests run in native code and always ask the modules for FOC; this one builds the module
 * requests itself so that FOC follows {@link RobotConstants#PRO_LICENSED}.
 *
 * <p>Runs on CTRE's odometry thread at the odometry rate, using the latest pose as the heading
 * measurement.
 */
final class FieldSpeedsRequest implements SwerveRequest {
    private final ModuleRequest moduleRequest = new ModuleRequest()
            .withDriveRequest(DriveRequestType.Velocity)
            .withSteerRequest(SteerRequestType.Position)
            .withEnableFOC(RobotConstants.PRO_LICENSED);
    private final PhoenixPIDController headingController;

    private volatile double vxMps;
    private volatile double vyMps;
    private volatile double omegaRadPerS;
    /** Heading to hold, or null to use {@link #omegaRadPerS}. */
    private volatile Rotation2d heading;

    private volatile double maxOmegaRadPerS;

    FieldSpeedsRequest(double headingKp, double headingKd) {
        headingController = new PhoenixPIDController(headingKp, 0, headingKd);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    FieldSpeedsRequest withSpeeds(double vx, double vy, double omega) {
        vxMps = vx;
        vyMps = vy;
        omegaRadPerS = omega;
        heading = null;
        return this;
    }

    FieldSpeedsRequest withHeading(double vx, double vy, Rotation2d target, double maxOmega) {
        vxMps = vx;
        vyMps = vy;
        maxOmegaRadPerS = maxOmega;
        heading = target;
        return this;
    }

    @Override
    public StatusCode apply(SwerveControlParameters p, SwerveModule<?, ?, ?>... modules) {
        Rotation2d robotHeading = p.currentPose.getRotation();
        double omega = omegaRadPerS;
        Rotation2d target = heading;
        if (target != null) {
            omega = headingController.calculate(robotHeading.getRadians(), target.getRadians(), p.timestamp);
            omega = MathUtil.clamp(omega, -maxOmegaRadPerS, maxOmegaRadPerS);
        }
        var robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(vxMps, vyMps, omega, robotHeading);
        robotRelative = ChassisSpeeds.discretize(robotRelative, p.updatePeriod);
        SwerveModuleState[] states = p.kinematics.toSwerveModuleStates(robotRelative, Translation2d.kZero);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, p.kMaxSpeedMps);
        moduleRequest.withUpdatePeriod(p.updatePeriod);
        for (int i = 0; i < modules.length; i++) {
            modules[i].apply(moduleRequest.withState(states[i]));
        }
        return StatusCode.OK;
    }
}
