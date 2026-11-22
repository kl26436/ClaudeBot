package frc.robot.field;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConstants.DriveK;

/**
 * Where a robot shoots algae into its alliance's net. Blue's net is on the upper half of the barge,
 * shot from the blue side facing +x; red's is blue's turned 180 degrees about the field centre.
 */
public final class Barge {
    private Barge() {}

    /** Where the front bumper stops for a net shot, on blue's side of the barge. */
    static final Translation2d BLUE_NET_LINE = new Translation2d(8.25, 6.18); // PLACEHOLDER(field-check)

    /** Half the width of the net along the barge. */
    public static final double NET_HALF_WIDTH_M = 1.8; // PLACEHOLDER(field-check)

    /** A point on the net line, facing the net: the robot's heading for a shot. */
    public static Pose2d netLine(Alliance a) {
        var blue = new Pose2d(BLUE_NET_LINE, Rotation2d.kZero);
        return a == Alliance.Blue ? blue : AllianceFlip.rotate(blue);
    }

    /** Robot pose for a net shot: front bumper on the net line, facing the net. */
    public static Pose2d netPose(Alliance a) {
        return netLine(a).transformBy(new Transform2d(
                -Units.inchesToMeters(DriveK.BUMPER_FACE_FROM_CENTRE_IN), 0, Rotation2d.kZero));
    }
}
