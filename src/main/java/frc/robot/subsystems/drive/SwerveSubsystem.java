// Adapted from FRC Team 2910's 2026CompetitionRobot-Public (MIT licence; see NOTICE).
package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.config.RobotConstants.DriveK;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain. In {@link Wanted#TELEOP} it follows the stick; in {@link
 * Wanted#DRIVE_TO_POINT} it drives a straight line to a target pose with {@link DriveToPoint} and
 * turns to the target's heading on the way.
 */
public class SwerveSubsystem extends SubsystemBase {
    public enum Wanted {
        IDLE,
        TELEOP,
        DRIVE_TO_POINT
    }

    /** Turn rate at full rotation stick. */
    static final double TELEOP_MAX_OMEGA_RAD_PER_S = 1.5 * 2 * Math.PI; // PLACEHOLDER(tuning)

    /** Turn rate limit while driving to a point. */
    static final double DRIVE_TO_POINT_MAX_OMEGA_RAD_PER_S = 2 * Math.PI; // PLACEHOLDER(tuning)

    static final double AT_TARGET_M = 0.03;
    static final double AT_TARGET_DEG = 2.0;

    private final SwerveIO io;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private final DriveToPoint autoController = new DriveToPoint(DriveToPoint.KP_AUTO, DriveToPoint.KD);
    private final DriveToPoint teleopController = new DriveToPoint(DriveToPoint.KP_TELEOP, DriveToPoint.KD);

    private Wanted wanted = Wanted.IDLE;
    private double stickX, stickY, stickOmega;
    private Pose2d target = new Pose2d();
    private double targetMaxSpeedMps;
    private boolean teleopGains;
    private double speedScale = 1.0;
    private DoubleSupplier speedScaleSource = null;
    private DoubleSupplier accelLimitSource = () -> Double.POSITIVE_INFINITY;
    /** The last field velocity sent, which the acceleration limit measures from. */
    private double lastVx = 0, lastVy = 0;

    /** Below this commanded speed an idle drivetrain is simply stopped. */
    static final double STOPPED_MPS = 0.02;
    private double lastPeriodicS = Double.NaN;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;
    }

    /** Stick input, each in -1..1, already turned to the blue-origin field frame for the alliance. */
    public void setTeleopInput(double x, double y, double omega) {
        stickX = MathUtil.clamp(x, -1, 1);
        stickY = MathUtil.clamp(y, -1, 1);
        stickOmega = MathUtil.clamp(omega, -1, 1);
        wanted = Wanted.TELEOP;
    }

    /** Drive straight to {@code target}, no faster than {@code maxSpeedMps} times the speed scale. */
    public void driveToPoint(Pose2d target, double maxSpeedMps, boolean teleopGains) {
        this.target = target;
        this.targetMaxSpeedMps = maxSpeedMps;
        this.teleopGains = teleopGains;
        wanted = Wanted.DRIVE_TO_POINT;
    }

    /** Stop and hold still until the next teleop input or drive-to-point target. */
    public void stopDriving() {
        wanted = Wanted.IDLE;
    }

    public Wanted wanted() {
        return wanted;
    }

    /** True while driving to a point and within 3 cm and 2 degrees of it. */
    public boolean atTarget() {
        return wanted == Wanted.DRIVE_TO_POINT
                && distanceToTargetM() <= AT_TARGET_M
                && Math.abs(inputs.pose.getRotation().minus(target.getRotation()).getDegrees()) <= AT_TARGET_DEG;
    }

    public double distanceToTargetM() {
        return inputs.pose.getTranslation().getDistance(target.getTranslation());
    }

    /** Scale teleop speed and the drive-to-point cap, for example while the elevator is up. */
    public void setSpeedScale(double scale) {
        speedScale = MathUtil.clamp(scale, 0, 1);
    }

    /**
     * Read the translation acceleration limit (m/s^2) from {@code source} every loop. Every commanded
     * change of field velocity, speeding up and slowing down alike, is held to it.
     */
    public void setAccelLimitSource(DoubleSupplier source) {
        accelLimitSource = source;
    }

    /** Read the speed scale from {@code source} at the start of every loop, before driving. */
    public void setSpeedScaleSource(DoubleSupplier source) {
        speedScaleSource = source;
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
        inputs.pose = pose;
    }

    public void resetTranslation(Translation2d t) {
        io.resetTranslation(t);
        inputs.pose = new Pose2d(t, inputs.pose.getRotation());
    }

    public Pose2d pose() {
        return inputs.pose;
    }

    /** Move the commanded field velocity toward (vx, vy) by no more than accelLimit * dt. */
    private void limit(double vx, double vy, double accelLimit, double dt) {
        double dvx = vx - lastVx;
        double dvy = vy - lastVy;
        double dv = Math.hypot(dvx, dvy);
        double maxDv = accelLimit * dt;
        if (Double.isFinite(maxDv) && dv > maxDv && dv > 0) {
            dvx *= maxDv / dv;
            dvy *= maxDv / dv;
        }
        lastVx += dvx;
        lastVy += dvy;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(inputs.speeds, inputs.pose.getRotation());
        RobotState.get().update(inputs.pose, fieldSpeeds);

        if (speedScaleSource != null) {
            setSpeedScale(speedScaleSource.getAsDouble());
        }

        double now = Timer.getFPGATimestamp();
        double dt = Double.isNaN(lastPeriodicS) ? 0 : now - lastPeriodicS;
        lastPeriodicS = now;

        double accelLimit = accelLimitSource.getAsDouble();
        double stepS = dt > 0 ? dt : 0.02;
        switch (wanted) {
            case IDLE -> {
                limit(0, 0, accelLimit, stepS);
                if (Math.hypot(lastVx, lastVy) < STOPPED_MPS) {
                    lastVx = 0;
                    lastVy = 0;
                    io.stop();
                } else {
                    io.driveFieldRelative(lastVx, lastVy, 0);
                }
            }
            case TELEOP -> {
                limit(stickX * DriveK.MAX_SPEED_MPS * speedScale, stickY * DriveK.MAX_SPEED_MPS * speedScale, accelLimit, stepS);
                io.driveFieldRelative(lastVx, lastVy, stickOmega * TELEOP_MAX_OMEGA_RAD_PER_S * speedScale);
            }
            case DRIVE_TO_POINT -> {
                var controller = teleopGains ? teleopController : autoController;
                var out = controller.calculate(
                        inputs.pose.getTranslation(), target.getTranslation(), targetMaxSpeedMps * speedScale, dt);
                limit(out.vxMps(), out.vyMps(), accelLimit, stepS);
                io.driveFieldRelativeFacing(lastVx, lastVy, target.getRotation(), DRIVE_TO_POINT_MAX_OMEGA_RAD_PER_S);
            }
        }
        Logger.recordOutput("Drive/AccelLimitMps2", accelLimit);

        Logger.recordOutput("Drive/Wanted", wanted);
        Logger.recordOutput("Drive/Target", target);
        Logger.recordOutput("Drive/AtTarget", atTarget());
        Logger.recordOutput("Drive/SpeedScale", speedScale);
    }
}
