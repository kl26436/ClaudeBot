package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Robot position from one AprilTag sighting plus the gyro heading. */
public final class SingleTagFix {
    private SingleTagFix() {}

    public static final double TAG_SIZE_M = Units.inchesToMeters(6.5);

    /**
     * Horizontal distance from the camera to the tag. With theta the angle the tag subtends
     * vertically, T the tag size and h the tag bottom height above the camera,
     * tan(theta) = T*d / (d^2 + h*(h+T)); this returns the larger root.
     */
    public static double distanceM(double heightPx, CameraModel cam, double tagBottomHeightM) {
        double theta = Math.toRadians(heightPx * cam.vFovDeg() / cam.vPixels());
        double tan = Math.tan(theta);
        double t = TAG_SIZE_M;
        double h = tagBottomHeightM - cam.robotToCamera().getZ();
        double disc = Math.max(0, t * t - 4 * tan * tan * h * (h + t));
        return (t + Math.sqrt(disc)) / (2 * tan);
    }

    public static Translation2d robotPosition(TagObservation obs, CameraModel cam, Rotation2d gyro, Pose3d tag) {
        double bottom = tag.getZ() - TAG_SIZE_M / 2;
        double d = distanceM(obs.heightPx(), cam, bottom);
        double camYaw = cam.robotToCamera().getRotation().getZ();
        // tx is positive to the right (clockwise), so the bearing is gyro + camera yaw - tx.
        var bearing = gyro.plus(Rotation2d.fromRadians(camYaw)).minus(Rotation2d.fromDegrees(obs.txDeg()));
        var tagXY = tag.toPose2d().getTranslation();
        var camera = tagXY.minus(new Translation2d(d, bearing));
        var mount = cam.robotToCamera().getTranslation().toTranslation2d();
        return camera.minus(mount.rotateBy(gyro));
    }
}
