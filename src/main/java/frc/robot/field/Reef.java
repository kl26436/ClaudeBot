package frc.robot.field;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConstants;
import java.util.ArrayList;
import java.util.List;

/** Reef scoring and algae poses. Faces are numbered 0..5 in the order AB, CD, EF, GH, IJ, KL. */
public final class Reef {
    private Reef() {}

    public enum Branch {
        A, B, C, D, E, F, G, H, I, J, K, L;

        public int face() {
            return ordinal() / 2;
        }

        /** Left as seen by a robot facing the reef face. */
        public boolean isLeft() {
            return ordinal() % 2 == 0;
        }
    }

    public static final double BRANCH_OFFSET_IN = 6.47;
    public static final double ALGAE_STANDOFF_IN = 0.5;

    /** Segments passing closer than this to the reef centre are blocked. */
    private static final double KEEP_OUT_RADIUS_M = 1.69;
    /** Radius of the circle that route via points sit on. */
    private static final double VIA_RADIUS_M = 1.75;
    private static final double VIA_FIRST_DEG = 30;
    private static final double VIA_STEP_DEG = 60;

    private static final int[] BLUE_TAGS = {18, 17, 22, 21, 20, 19};
    private static final int[] RED_TAGS = {7, 8, 9, 10, 11, 6};

    private static final Translation2d BLUE_CENTRE = blueCentre();

    private static Translation2d blueCentre() {
        double x = 0, y = 0;
        for (int id : BLUE_TAGS) {
            var t = Tags.pose(id).getTranslation();
            x += t.getX();
            y += t.getY();
        }
        return new Translation2d(x / BLUE_TAGS.length, y / BLUE_TAGS.length);
    }

    private static Translation2d centre(Alliance a) {
        return a == Alliance.Blue ? BLUE_CENTRE : AllianceFlip.rotate(BLUE_CENTRE);
    }

    public static Pose2d tagPose(Alliance a, int face) {
        return Tags.pose((a == Alliance.Blue ? BLUE_TAGS : RED_TAGS)[face]);
    }

    /** Robot facing the tag, `outIn` inches out along the tag normal, `leftIn` inches to the robot's left. */
    private static Pose2d facing(Pose2d tag, double outIn, double leftIn) {
        // the robot looks back at the tag, so its left is the tag frame's -y
        return tag.transformBy(new Transform2d(
                Units.inchesToMeters(outIn), -Units.inchesToMeters(leftIn), Rotation2d.k180deg));
    }

    public static Pose2d scoringPose(Alliance a, Branch b) {
        double left = b.isLeft() ? BRANCH_OFFSET_IN : -BRANCH_OFFSET_IN;
        return facing(tagPose(a, b.face()), RobotConstants.DriveK.BUMPER_FACE_FROM_CENTRE_IN, left);
    }

    public static Pose2d algaePose(Alliance a, int face) {
        return facing(tagPose(a, face), RobotConstants.DriveK.BUMPER_FACE_FROM_CENTRE_IN + ALGAE_STANDOFF_IN, 0);
    }

    public static boolean algaeIsHigh(int face) {
        return face % 2 == 0;
    }

    public static int nearestFace(Alliance a, Translation2d robot) {
        int best = 0;
        double bestDist = Double.MAX_VALUE;
        for (int f = 0; f < 6; f++) {
            double d = tagPose(a, f).getTranslation().getDistance(robot);
            if (d < bestDist) {
                bestDist = d;
                best = f;
            }
        }
        return best;
    }

    /**
     * True when the segment does not cut into the keep-out circle around the reef centre. A segment that
     * starts inside the circle is clear if it only moves away from the centre (driving out of a scoring pose).
     */
    public static boolean pathClear(Alliance a, Translation2d from, Translation2d to) {
        var c = centre(a);
        var d = to.minus(from);
        double len2 = d.getX() * d.getX() + d.getY() * d.getY();
        var rel = from.minus(c);
        double t = len2 == 0 ? 0 : -(rel.getX() * d.getX() + rel.getY() * d.getY()) / len2;
        t = Math.max(0, Math.min(1, t));
        if (t == 0 && rel.getNorm() < KEEP_OUT_RADIUS_M) {
            // closest approach is the start point, so the segment never gets nearer than where it begins
            return true;
        }
        var closest = new Translation2d(from.getX() + t * d.getX(), from.getY() + t * d.getY());
        return closest.getDistance(c) >= KEEP_OUT_RADIUS_M;
    }

    /**
     * Via points that walk round the reef, then {@code to}. Corner via points sit at 30 + 60k degrees on the via
     * circle; between two corners a mid-face point is added so every hop stays outside the keep-out circle.
     */
    public static List<Pose2d> route(Alliance a, Translation2d from, Pose2d to) {
        var out = new ArrayList<Pose2d>();
        if (pathClear(a, from, to.getTranslation())) {
            out.add(to);
            return out;
        }
        var c = centre(a);
        double angFrom = Math.toDegrees(Math.atan2(from.getY() - c.getY(), from.getX() - c.getX()));
        double angTo = Math.toDegrees(Math.atan2(to.getY() - c.getY(), to.getX() - c.getX()));
        double ccw = Math.floorMod(Math.round((angTo - angFrom) * 1e6), 360_000_000L) / 1e6;
        boolean goCcw = ccw <= 180;
        double span = goCcw ? ccw : 360 - ccw;
        double dir = goCcw ? 1 : -1;

        // corner angles (relative to the start angle) strictly inside the arc, in travel order
        var corners = new ArrayList<Double>();
        for (int k = -6; k <= 12; k++) {
            double corner = VIA_FIRST_DEG + VIA_STEP_DEG * k;
            double travelled = Math.floorMod(Math.round(dir * (corner - angFrom) * 1e6), 360_000_000L) / 1e6;
            if (travelled > 1e-6 && travelled < span - 1e-6) {
                corners.add(travelled);
            }
        }
        corners.sort(Double::compare);
        if (corners.isEmpty()) {
            // same sector but still blocked: step round via the nearer corner
            corners.add(span / 2);
        }
        var heading = to.getRotation();
        for (int i = 0; i < corners.size(); i++) {
            if (i > 0 && corners.get(i) - corners.get(i - 1) > 45) {
                out.add(via(c, angFrom + dir * (corners.get(i) + corners.get(i - 1)) / 2, heading));
            }
            out.add(via(c, angFrom + dir * corners.get(i), heading));
        }
        out.add(to);
        return out;
    }

    private static Pose2d via(Translation2d centre, double angDeg, Rotation2d heading) {
        var offset = new Translation2d(VIA_RADIUS_M, Rotation2d.fromDegrees(angDeg));
        return new Pose2d(centre.plus(offset), heading);
    }
}
