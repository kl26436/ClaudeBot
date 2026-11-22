// Adapted from FRC Team 2910's 2026CompetitionRobot-Public (MIT licence; see NOTICE).
package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the swerve drivetrain. Field poses are blue-origin, in metres and radians. */
public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        /** Odometry estimate of the robot on the field. */
        public Pose2d pose = new Pose2d();
        /** Measured chassis speeds, robot-relative. */
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public SwerveModuleState[] moduleStates = emptyStates();
        public SwerveModuleState[] moduleTargets = emptyStates();
        public Rotation2d gyroYaw = new Rotation2d();
        /** True while odometry samples are arriving from every module and the gyro. */
        public boolean connected;

        private static SwerveModuleState[] emptyStates() {
            return new SwerveModuleState[] {
                new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
            };
        }
    }

    default void updateInputs(SwerveIOInputs in) {}

    /** Field-relative velocity command, blue-origin. */
    default void driveFieldRelative(double vxMps, double vyMps, double omegaRadPerS) {}

    /** Field-relative translation while turning to, then holding, a field heading. */
    default void driveFieldRelativeFacing(double vxMps, double vyMps, Rotation2d heading, double maxOmegaRadPerS) {}

    default void resetPose(Pose2d pose) {}

    /** Move the estimate to a new position on the field, keeping its heading. */
    default void resetTranslation(Translation2d t) {}

    default void stop() {}
}
