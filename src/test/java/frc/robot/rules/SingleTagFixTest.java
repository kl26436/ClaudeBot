package frc.robot.rules;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.config.RobotConstants;
import frc.robot.field.Reef;
import frc.robot.subsystems.vision.CameraModel;
import frc.robot.subsystems.vision.SingleTagFix;
import java.io.IOException;
import java.util.Random;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class SingleTagFixTest {
    private static AprilTagFieldLayout layout;

    @BeforeAll static void load() throws IOException {
        layout = AprilTagFieldLayout.loadField(RobotConstants.FIELD_LAYOUT);
    }

    @ParameterizedTest @ValueSource(doubles = {0.6, 1.0, 2.0, 3.0})
    void recoversAKnownPose(double metresOut) {
        var tag = layout.getTagPose(18).get(); var cam = CameraModel.limelight4();
        var truth = Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(metresOut, 0.2, Rotation2d.k180deg));
        var obs = cam.project(truth, 18, tag, 0, new Random(1)).orElseThrow();
        var fix = SingleTagFix.robotPosition(obs, cam, truth.getRotation(), tag);
        assertEquals(0, fix.getDistance(truth.getTranslation()), metresOut <= 1.0 ? 0.03 : 0.08);
    }

    @Test void tagBehindTheCameraIsNotSeen() {
        var tag = layout.getTagPose(18).get(); var cam = CameraModel.limelight4();
        var facing = Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(1.5, 0, Rotation2d.k180deg));
        assertTrue(cam.project(facing, 18, tag, 0, new Random(1)).isPresent(), "control: seen when facing it");
        var away = new edu.wpi.first.math.geometry.Pose2d(facing.getTranslation(), facing.getRotation().plus(Rotation2d.k180deg));
        assertTrue(cam.project(away, 18, tag, 0, new Random(1)).isEmpty());
    }

    @Test void tagFacingAwayOrTooFarOrOffAxisIsNotSeen() {
        var tag = layout.getTagPose(18).get(); var cam = CameraModel.limelight4();
        // Behind the tag, looking at its back.
        var behindTag = Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(-1.5, 0, Rotation2d.kZero));
        assertTrue(cam.project(behindTag, 18, tag, 0, new Random(1)).isEmpty());
        var far = Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(4.5, 0, Rotation2d.k180deg));
        assertTrue(cam.project(far, 18, tag, 0, new Random(1)).isEmpty());
        var turned = Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(1.5, 0, Rotation2d.fromDegrees(180 + 60)));
        assertTrue(cam.project(turned, 18, tag, 0, new Random(1)).isEmpty());
    }

    @Test void txIsPositiveWhenTheTagIsToTheRight() {
        var tag = layout.getTagPose(18).get(); var cam = CameraModel.limelight4();
        var tagPose = Reef.tagPose(Alliance.Blue, 0);
        // The robot faces the tag, so its right is the tag frame's +y: sit at -y and the tag is on the right.
        var tagOnRight = tagPose.transformBy(new Transform2d(1.5, -0.3, Rotation2d.k180deg));
        assertTrue(tagPose.relativeTo(tagOnRight).getY() < 0, "geometry check: tag is right of the robot");
        assertTrue(cam.project(tagOnRight, 18, tag, 0, new Random(1)).orElseThrow().txDeg() > 0);
        var tagOnLeft = tagPose.transformBy(new Transform2d(1.5, 0.3, Rotation2d.k180deg));
        assertTrue(tagPose.relativeTo(tagOnLeft).getY() > 0, "geometry check: tag is left of the robot");
        assertTrue(cam.project(tagOnLeft, 18, tag, 0, new Random(1)).orElseThrow().txDeg() < 0);
    }

    @Test void cameraLooksUp() {
        var forward = new Translation3d(1, 0, 0).rotateBy(CameraModel.limelight4().robotToCamera().getRotation());
        assertTrue(forward.getZ() > 0);
        assertEquals(Math.sin(Math.toRadians(12)), forward.getZ(), 1e-9);
    }
}
