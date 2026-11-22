package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.RobotConstants.DriveK;

/**
 * Drives the robot in a straight line toward a field point with a PD loop on the straight-line
 * distance to the target. No path following.
 */
public final class DriveToPoint {
    public record Output(double vxMps, double vyMps) {}

    public static final double KP_AUTO = 3.0;
    public static final double KP_TELEOP = 3.6;
    public static final double KD = 0.1;
    public static final double FRICTION_FRACTION = 0.02;
    public static final double DONE_M = 0.01;

    private final double kP;
    private final double kD;

    private Translation2d lastTarget;
    private double lastDistanceM = Double.NaN;

    public DriveToPoint(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    /**
     * Returns the field-relative velocity command toward {@code target}.
     *
     * <p>The derivative term is zero on the first call and whenever the target changes, because
     * there is no previous distance for that target. It is also zero when {@code dtSeconds} is not
     * positive. The PD term is floored at zero, so a robot arriving faster than the loop wants
     * coasts on the friction term instead of being sped up by a negative derivative.
     */
    public Output calculate(Translation2d current, Translation2d target, double maxSpeedMps, double dtSeconds) {
        Translation2d error = target.minus(current);
        double distanceM = error.getNorm();

        double derivative = 0.0;
        if (target.equals(lastTarget) && !Double.isNaN(lastDistanceM) && dtSeconds > 0.0) {
            derivative = (distanceM - lastDistanceM) / dtSeconds;
        }
        lastTarget = target;
        lastDistanceM = distanceM;

        if (distanceM < DONE_M) {
            return new Output(0.0, 0.0);
        }

        double pd = kP * distanceM + kD * derivative;
        double speed = Math.min(Math.max(0.0, pd) + FRICTION_FRACTION * DriveK.MAX_SPEED_MPS, maxSpeedMps);
        Translation2d direction = error.div(distanceM);
        return new Output(direction.getX() * speed, direction.getY() * speed);
    }
}
