package frc.robot.testutil;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;

public final class PoseAsserts {
    private PoseAsserts() {}

    /** Translation within tol metres and heading within tol radians (wrap-safe). */
    public static void assertPoseEquals(Pose2d expected, Pose2d actual, double tol) {
        assertEquals(0, expected.getTranslation().getDistance(actual.getTranslation()), tol, "translation");
        assertEquals(0, expected.getRotation().minus(actual.getRotation()).getRadians(), tol, "heading");
    }
}
