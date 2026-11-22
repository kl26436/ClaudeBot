package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.RobotConstants.DriveK;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.Setpoints;
import frc.robot.config.Setpoints.Pose;
import frc.robot.field.Barge;
import frc.robot.field.Reef;
import frc.robot.subsystems.Superstructure.Wanted;
import java.util.Optional;

/**
 * Decides, from where the robot really is and where its mechanism really points, whether a released
 * game piece scored. Pure geometry: the simulation calls it on the tick a piece leaves the robot.
 */
public final class ScoreJudge {
    private ScoreJudge() {}

    public record CoralScore(Alliance alliance, Reef.Branch branch, Wanted level, double timeS) {}

    /** How far from a branch's scoring pose the robot may be. */
    static final double POSE_TOL_IN = 1.5;

    static final double HEADING_TOL_DEG = 2.0;

    /** How far from the branch point the wrist's line may pass. */
    static final double LINE_TOL_IN = 1.0;

    /** Branch points in the robot frame at the scoring pose, inches forward of centre and up from the carpet. */
    record BranchPoint(Wanted level, double forwardIn, double upIn) {}

    static final BranchPoint[] BRANCH_POINTS = {
        new BranchPoint(Wanted.L2, 19.025, 31.875),
        new BranchPoint(Wanted.L3, 19.025, 47.625),
        new BranchPoint(Wanted.L4, 18.525, 72.0),
    };

    /** How far the front bumper may be from the net line, and the heading tolerance for a net shot. */
    static final double NET_LINE_TOL_M = 0.3; // PLACEHOLDER(field-check)

    static final double NET_HEADING_TOL_DEG = 10;

    /**
     * The branch and level a coral released now lands on, or empty. The robot must be on a branch's
     * scoring pose; L1 needs the L1 pose, and L2 to L4 need the wrist's line, from the pivot at the
     * wrist angle, to pass within {@link #LINE_TOL_IN} of that level's branch point.
     */
    public static Optional<CoralScore> judgeCoral(
            Alliance a, Pose2d truePose, double carriageIn, double wristDeg, double timeS) {
        Optional<Reef.Branch> branch = branchAt(a, truePose);
        if (branch.isEmpty()) {
            return Optional.empty();
        }
        if (Math.abs(carriageIn - Pose.L1.carriageIn) <= Setpoints.CARRIAGE_TOL_IN
                && Math.abs(wristDeg - Pose.L1.wristDeg) <= Setpoints.WRIST_TOL_DEG) {
            return Optional.of(new CoralScore(a, branch.get(), Wanted.L1, timeS));
        }
        double pivotX = ElevatorK.PIVOT_X_IN;
        double pivotZ = ElevatorK.PIVOT_REST_HEIGHT_IN + carriageIn;
        double dx = Math.cos(Math.toRadians(wristDeg));
        double dz = Math.sin(Math.toRadians(wristDeg));
        BranchPoint best = null;
        double bestMiss = Double.POSITIVE_INFINITY;
        for (BranchPoint p : BRANCH_POINTS) {
            double rx = p.forwardIn() - pivotX;
            double rz = p.upIn() - pivotZ;
            boolean ahead = rx * dx + rz * dz > 0;
            double miss = Math.abs(rx * dz - rz * dx);
            if (ahead && miss <= LINE_TOL_IN && miss < bestMiss) {
                best = p;
                bestMiss = miss;
            }
        }
        return best == null ? Optional.empty() : Optional.of(new CoralScore(a, branch.get(), best.level(), timeS));
    }

    /** The branch whose scoring pose the robot is on, within tolerance. */
    static Optional<Reef.Branch> branchAt(Alliance a, Pose2d truePose) {
        for (Reef.Branch b : Reef.Branch.values()) {
            Pose2d p = Reef.scoringPose(a, b);
            if (Units.metersToInches(truePose.getTranslation().getDistance(p.getTranslation())) <= POSE_TOL_IN
                    && Math.abs(truePose.getRotation().minus(p.getRotation()).getDegrees()) <= HEADING_TOL_DEG) {
                return Optional.of(b);
            }
        }
        return Optional.empty();
    }

    /**
     * Whether an algae released now goes into the alliance's net: the mechanism at the net pose, the
     * robot facing the net, and its front bumper within {@link #NET_LINE_TOL_M} of the net line.
     */
    public static boolean judgeNet(Alliance a, Pose2d truePose, double carriageIn, double wristDeg) {
        if (Math.abs(carriageIn - Pose.NET.carriageIn) > Setpoints.CARRIAGE_TOL_IN
                || Math.abs(wristDeg - Pose.NET.wristDeg) > Setpoints.WRIST_TOL_DEG) {
            return false;
        }
        Pose2d line = Barge.netLine(a);
        if (Math.abs(truePose.getRotation().minus(line.getRotation()).getDegrees()) > NET_HEADING_TOL_DEG) {
            return false;
        }
        Translation2d front = truePose.getTranslation().plus(new Translation2d(
                Units.inchesToMeters(DriveK.BUMPER_FACE_FROM_CENTRE_IN), truePose.getRotation()));
        Translation2d rel = front.minus(line.getTranslation()).rotateBy(line.getRotation().unaryMinus());
        return Math.abs(rel.getX()) <= NET_LINE_TOL_M && Math.abs(rel.getY()) <= Barge.NET_HALF_WIDTH_M;
    }
}
