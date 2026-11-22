package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConstants;
import java.util.Optional;
import java.util.Random;

/** Pinhole-style camera used by the simulator to produce the sightings a Limelight would report. */
public record CameraModel(Transform3d robotToCamera, double hFovDeg, double vFovDeg, int hPixels, int vPixels) {
    public static final double MAX_RANGE_M = 4.0;

    public static CameraModel limelight4() {
        // WPILib pitch is positive nose-down, so a camera tilted up has a negative pitch.
        var mount = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(RobotConstants.CameraK.FORWARD_IN),
                        0,
                        Units.inchesToMeters(RobotConstants.CameraK.HEIGHT_IN)),
                new Rotation3d(0, -Math.toRadians(RobotConstants.CameraK.PITCH_UP_DEG), 0));
        return new CameraModel(
                mount,
                RobotConstants.CameraK.H_FOV_DEG,
                RobotConstants.CameraK.V_FOV_DEG,
                (int) RobotConstants.CameraK.H_PIXELS,
                (int) RobotConstants.CameraK.V_PIXELS);
    }

    /**
     * What the camera reports for a tag, or empty if the tag is more than 4 m away, outside half the
     * horizontal field of view, or facing away from the camera.
     */
    public Optional<TagObservation> project(Pose2d robot, int tagId, Pose3d tag, double noisePx, Random rng) {
        var camera = new Pose3d(robot).transformBy(robotToCamera);
        var cam = camera.getTranslation();
        var centre = tag.getTranslation();

        // The tag's +x axis is its outward normal; it must point toward the camera.
        var normal = new Translation3d(1, 0, 0).rotateBy(tag.getRotation());
        var toCamera = cam.minus(centre);
        if (normal.getX() * toCamera.getX() + normal.getY() * toCamera.getY() + normal.getZ() * toCamera.getZ() <= 0) {
            return Optional.empty();
        }

        double dx = centre.getX() - cam.getX();
        double dy = centre.getY() - cam.getY();
        double range = Math.hypot(dx, dy);
        if (Math.hypot(range, centre.getZ() - cam.getZ()) > MAX_RANGE_M) return Optional.empty();

        // Bearing relative to the camera's heading, left positive; tx is right positive.
        double camYaw = camera.getRotation().getZ();
        double bearing = Math.atan2(dy, dx) - camYaw;
        bearing = Math.atan2(Math.sin(bearing), Math.cos(bearing));
        // Behind the camera: the cosine test below also rejects it via the FOV check.
        if (Math.abs(Math.toDegrees(bearing)) > hFovDeg / 2) return Optional.empty();
        double txDeg = -Math.toDegrees(bearing);

        double half = SingleTagFix.TAG_SIZE_M / 2;
        double elevTop = Math.atan2(centre.getZ() + half - cam.getZ(), range);
        double elevBottom = Math.atan2(centre.getZ() - half - cam.getZ(), range);
        double heightPx = Math.toDegrees(elevTop - elevBottom) * vPixels / vFovDeg;

        if (noisePx > 0) {
            heightPx += rng.nextGaussian() * noisePx;
            txDeg += rng.nextGaussian() * noisePx * hFovDeg / hPixels;
        }
        return Optional.of(new TagObservation(tagId, txDeg, heightPx));
    }
}
