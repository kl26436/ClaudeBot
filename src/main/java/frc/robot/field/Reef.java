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
    private static final double VIA_RADIUS_M = 2.0;
    private static final double VIA_FIRST_DEG = 30;
    private static final double VIA_STEP_DEG = 60;

    private static final int[] BLUE_TAGS = {18, 17, 22, 21, 20, 19};
    private static final int[] RED_TAGS = {7, 8, 9, 10, 11, 6};

    private static final Translation2d BLUE_CENTRE = centreOf(BLUE_TAGS);
    private static final Translation2d RED_CENTRE = centreOf(RED_TAGS);

    /** Mean of the six face tags: the layout's own reef centre, so corner bearings line up with the tags. */
    private static Translation2d centreOf(int[] tags) {
        double x = 0, y = 0;
        for (int id : tags) {
            var t = Tags.pose(id).getTranslation();
            x += t.getX();
            y += t.getY();
        }
        return new Translation2d(x / tags.length, y / tags.length);
    }

    private static Translation2d centre(Alliance a) {
        return a == Alliance.Blue ? BLUE_CENTRE : RED_CENTRE;
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
     * False only when the segment passes by the reef: its closest point to the centre is strictly inside the
     * segment and nearer than the keep-out radius. Leaving from, or arriving at, a pose inside the circle is allowed.
     */
    public static boolean pathClear(Alliance a, Translation2d from, Translation2d to) {
        var c = centre(a);
        var d = to.minus(from);
        double len2 = d.getX() * d.getX() + d.getY() * d.getY();
        if (len2 == 0) {
            return true;
        }
        var rel = c.minus(from);
        double t = (rel.getX() * d.getX() + rel.getY() * d.getY()) / len2;
        if (t <= 0 || t >= 1) {
            return true;
        }
        var closest = new Translation2d(from.getX() + t * d.getX(), from.getY() + t * d.getY());
        return closest.getDistance(c) >= KEEP_OUT_RADIUS_M;
    }

    /** Via points round the reef corners (30 + 60k degrees on the via circle), then {@code to}. */
    public static List<Pose2d> route(Alliance a, Translation2d from, Pose2d to) {
        var direct = List.of(to);
        if (pathClear(a, from, to.getTranslation())
                || nearestFace(a, from) == nearestFace(a, to.getTranslation())) {
            return direct;
        }
        var c = centre(a);
        double angFrom = bearing(c, from);
        double angTo = bearing(c, to.getTranslation());
        double ccw = Math.floorMod(Math.round((angTo - angFrom) * 1e6), 360_000_000L) / 1e6;
        boolean goCcw = ccw <= 180;
        double span = goCcw ? ccw : 360 - ccw;
        double dir = goCcw ? 1 : -1;

        var corners = new ArrayList<Double>(); // bearings strictly between from and to, in travel order
        for (int k = 0; k < 6; k++) {
            double corner = VIA_FIRST_DEG + VIA_STEP_DEG * k;
            double travelled = Math.floorMod(Math.round(dir * (corner - angFrom) * 1e6), 360_000_000L) / 1e6;
            if (travelled <= span + 1e-6) {
                corners.add(travelled);
            }
        }
        if (corners.isEmpty()) {
            return direct;
        }
        corners.sort(Double::compare);
        var heading = to.getRotation();
        var out = new ArrayList<Pose2d>();
        var first = via(c, angFrom + dir * corners.get(0), heading);
        if (!pathClear(a, from, first.getTranslation())) {
            out.add(via(c, angFrom, heading)); // back straight out
        }
        for (double t : corners) {
            out.add(via(c, angFrom + dir * t, heading));
        }
        if (!pathClear(a, out.get(out.size() - 1).getTranslation(), to.getTranslation())) {
            out.add(via(c, angTo, heading)); // come straight in
        }
        out.add(to);
        return out;
    }

    private static double bearing(Translation2d centre, Translation2d p) {
        return Math.toDegrees(Math.atan2(p.getY() - centre.getY(), p.getX() - centre.getX()));
    }

    private static Pose2d via(Translation2d centre, double angDeg, Rotation2d heading) {
        var offset = new Translation2d(VIA_RADIUS_M, Rotation2d.fromDegrees(angDeg));
        return new Pose2d(centre.plus(offset), heading);
    }
}
