package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.vision.VisionIOLimelight;
import org.junit.jupiter.api.Test;

class LimelightCornersTest {
    @Test void squareTagHeightIsItsSide() {
        // Corners in Limelight order are not guaranteed, so shuffle them.
        double[] xy = {140, 60, 100, 100, 100, 60, 140, 100};
        assertEquals(40, VisionIOLimelight.tagHeightPx(xy), 1e-9);
    }

    @Test void aTagSeenFromTheSideAveragesItsEdges() {
        // Near edge 50 px tall, far edge 30 px tall.
        double[] xy = {100, 50, 100, 100, 130, 60, 130, 90};
        assertEquals(40, VisionIOLimelight.tagHeightPx(xy), 1e-9);
    }

    @Test void tooFewCornersIsNaN() {
        assertTrue(Double.isNaN(VisionIOLimelight.tagHeightPx(new double[] {1, 2, 3, 4})));
    }
}
