package frc.robot.field;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Auto starting poses on the blue side; red is the blue pose rotated about the field centre. */
public enum StartPose {
    LEFT(6.05), // PLACEHOLDER(field-check)
    CENTRE(4.026), // PLACEHOLDER(field-check)
    RIGHT(2.0); // PLACEHOLDER(field-check)

    private static final double BLUE_X_M = 7.2; // PLACEHOLDER(field-check)

    private final double blueY;

    StartPose(double blueY) {
        this.blueY = blueY;
    }

    public Pose2d pose(Alliance a) {
        var blue = new Pose2d(BLUE_X_M, blueY, Rotation2d.k180deg);
        return a == Alliance.Blue ? blue : AllianceFlip.rotate(blue);
    }
}
