package frc.robot.field;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConstants;

/** Coral station pickup poses. Sides are as seen by the driver looking down the field. */
public final class Station {
    private Station() {}

    public enum Side { LEFT, RIGHT }

    private static int tagId(Alliance a, Side s) {
        if (a == Alliance.Blue) {
            return s == Side.RIGHT ? 12 : 13;
        }
        return s == Side.RIGHT ? 2 : 1;
    }

    /** Heading equals the tag's, so the robot's rear faces the wall; centre one bumper-face out from the tag. */
    public static Pose2d pickupPose(Alliance a, Side s) {
        var tag = Tags.pose(tagId(a, s));
        return tag.transformBy(new Transform2d(
                Units.inchesToMeters(RobotConstants.DriveK.BUMPER_FACE_FROM_CENTRE_IN), 0, Rotation2d.kZero));
    }

    public static Side nearest(Alliance a, Translation2d robot) {
        double left = pickupPose(a, Side.LEFT).getTranslation().getDistance(robot);
        double right = pickupPose(a, Side.RIGHT).getTranslation().getDistance(robot);
        return left <= right ? Side.LEFT : Side.RIGHT;
    }
}
