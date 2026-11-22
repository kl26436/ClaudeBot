package frc.robot.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.RobotConstants;

/** The single loaded copy of the AprilTag layout, shared by the field classes. */
final class Tags {
    private Tags() {}

    static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(RobotConstants.FIELD_LAYOUT);

    static Pose2d pose(int id) {
        return LAYOUT.getTagPose(id).orElseThrow().toPose2d();
    }
}
