package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Maps blue-alliance field geometry to red. The 2025 welded field is 180-degree rotationally symmetric. */
public final class AllianceFlip {
    private AllianceFlip() {}

    public static Translation2d rotate(Translation2d blue) {
        return new Translation2d(Tags.LAYOUT.getFieldLength() - blue.getX(), Tags.LAYOUT.getFieldWidth() - blue.getY());
    }

    public static Pose2d rotate(Pose2d blue) {
        return new Pose2d(rotate(blue.getTranslation()), blue.getRotation().plus(Rotation2d.k180deg));
    }
}
