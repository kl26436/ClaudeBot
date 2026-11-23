package frc.robot.rules;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.testutil.PoseAsserts.assertPoseEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.util.Units;
import frc.robot.field.AllianceFlip;
import frc.robot.field.Station;
import org.junit.jupiter.api.Test;

class StationTest {
    @Test void stationRearFacesWall() {
        // blue right = tag 12, blue left = tag 13; red right = 2, red left = 1
        int[][] ids = {{12, 13}, {2, 1}};
        var layout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(frc.robot.config.RobotConstants.FIELD_LAYOUT);
        for (var a : Alliance.values()) for (var s : Station.Side.values()) {
            var tag = layout.getTagPose(ids[a == Alliance.Blue ? 0 : 1][s.ordinal() == 1 ? 0 : 1]).orElseThrow().toPose2d();
            var pick = Station.pickupPose(a, s);
            assertEquals(0, pick.getRotation().minus(tag.getRotation()).getRadians(), 1e-9, "heading equals tag heading");
            var rel = pick.relativeTo(tag);
            assertEquals(Units.inchesToMeters(17.25), rel.getX(), 1e-6);
            assertEquals(0, rel.getY(), 1e-6);
        }
        assertPoseEquals(AllianceFlip.rotate(Station.pickupPose(Alliance.Blue, Station.Side.RIGHT)), Station.pickupPose(Alliance.Red, Station.Side.RIGHT), 1e-3);
        assertPoseEquals(AllianceFlip.rotate(Station.pickupPose(Alliance.Blue, Station.Side.LEFT)), Station.pickupPose(Alliance.Red, Station.Side.LEFT), 1e-3);
    }

    @Test void nearestPicksTheCloserSide() {
        for (var a : Alliance.values()) for (var s : Station.Side.values())
            assertEquals(s, Station.nearest(a, Station.pickupPose(a, s).getTranslation()));
    }
}
