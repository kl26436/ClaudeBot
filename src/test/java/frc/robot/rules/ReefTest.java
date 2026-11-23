package frc.robot.rules;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.testutil.PoseAsserts.assertPoseEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.field.AllianceFlip;
import frc.robot.field.Reef;
import frc.robot.field.StartPose;
import frc.robot.field.Station;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ReefTest {
    @Test void branchPosesSitSquareAtBumperDistance() {
        for (var a : Alliance.values()) for (var b : Reef.Branch.values()) {
            var tag = Reef.tagPose(a, b.face()); var robot = Reef.scoringPose(a, b);
            var rel = robot.relativeTo(tag);
            assertEquals(Units.inchesToMeters(17.25), rel.getX(), 1e-6);
            assertEquals(Units.inchesToMeters(6.47), Math.abs(rel.getY()), 1e-6);
            assertEquals(180, Math.abs(rel.getRotation().getDegrees()), 1e-6);
        }
    }

    @Test void leftBranchIsOnTheRobotsLeft() {
        var left = Reef.scoringPose(Alliance.Blue, Reef.Branch.A); var right = Reef.scoringPose(Alliance.Blue, Reef.Branch.B);
        assertTrue(Reef.Branch.A.isLeft()); assertFalse(Reef.Branch.B.isLeft());
        assertTrue(right.relativeTo(left).getY() < 0);
    }

    @Test void redIsBlueRotated() {
        for (var b : Reef.Branch.values()) assertPoseEquals(AllianceFlip.rotate(Reef.scoringPose(Alliance.Blue, b)), Reef.scoringPose(Alliance.Red, b), 1e-3);
    }

    @Test void faceAbFacesTheAllianceWall() {
        assertEquals(180, Math.abs(Reef.tagPose(Alliance.Blue, 0).getRotation().getDegrees()), 1e-6);
    }

    @Test void routesClearTheReef() {
        for (var a : Alliance.values()) for (var b : Reef.Branch.values()) for (var side : Station.Side.values()) {
            var from = Reef.scoringPose(a, b).getTranslation();
            var to = Station.pickupPose(a, side);
            var pts = Reef.route(a, from, to); var prev = from;
            for (var p : pts) { assertTrue(Reef.pathClear(a, prev, p.getTranslation()), a + " " + b + " " + side + " " + p); prev = p.getTranslation(); }
            assertEquals(to, pts.get(pts.size() - 1));
            for (var p : pts) assertEquals(to.getRotation(), p.getRotation());
        }
    }

    @Test void pathThroughTheReefIsBlocked() {
        var c = Reef.tagPose(Alliance.Blue, 0).getTranslation().plus(Reef.tagPose(Alliance.Blue, 3).getTranslation()).div(2);
        assertFalse(Reef.pathClear(Alliance.Blue, c.plus(new Translation2d(-3, 0)), c.plus(new Translation2d(3, 0))));
        assertTrue(Reef.pathClear(Alliance.Blue, c.plus(new Translation2d(-3, 3)), c.plus(new Translation2d(3, 3))));
    }

    @Test void algaeFacesAndHeights() {
        for (int f = 0; f < 6; f++) {
            var rel = Reef.algaePose(Alliance.Blue, f).relativeTo(Reef.tagPose(Alliance.Blue, f));
            assertEquals(Units.inchesToMeters(17.75), rel.getX(), 1e-6);
            assertEquals(0, rel.getY(), 1e-6);
            assertEquals(f % 2 == 0, Reef.algaeIsHigh(f));
        }
        assertEquals(3, Reef.nearestFace(Alliance.Blue, Reef.tagPose(Alliance.Blue, 3).getTranslation()));
    }

    @Test void startPosesMirrorForRed() {
        for (var s : StartPose.values()) assertPoseEquals(AllianceFlip.rotate(s.pose(Alliance.Blue)), s.pose(Alliance.Red), 1e-9);
        assertEquals(180, Math.abs(StartPose.LEFT.pose(Alliance.Blue).getRotation().getDegrees()), 1e-9);
    }

    private static Translation2d centre(Alliance a) {
        Translation2d sum = Translation2d.kZero;
        for (int f = 0; f < 6; f++) sum = sum.plus(Reef.tagPose(a, f).getTranslation());
        return sum.div(6);
    }

    @Test void arrivingAndLeavingAreClearButPassingIsNot() {
        var a = Alliance.Blue;
        var out = Reef.tagPose(a, 0).getTranslation().plus(new Translation2d(3.0, Reef.tagPose(a, 0).getRotation()));
        var scoreA = Reef.scoringPose(a, Reef.Branch.A).getTranslation();
        assertTrue(Reef.pathClear(a, out, scoreA));
        assertTrue(Reef.pathClear(a, scoreA, out));
        assertFalse(Reef.pathClear(a, scoreA, Reef.scoringPose(a, Reef.Branch.G).getTranslation()));
        assertFalse(Reef.pathClear(a, scoreA, Reef.scoringPose(a, Reef.Branch.B).getTranslation()));
    }

    @Test void sameFaceMovesGoDirect() {
        var a = Alliance.Blue;
        var from = Reef.scoringPose(a, Reef.Branch.A).getTranslation();
        assertEquals(List.of(Reef.algaePose(a, 0)), Reef.route(a, from, Reef.algaePose(a, 0)));
        assertEquals(List.of(Reef.scoringPose(a, Reef.Branch.B)), Reef.route(a, from, Reef.scoringPose(a, Reef.Branch.B)));
    }

    @Test void adjacentFaceGoesRoundOneCorner() {
        var a = Alliance.Blue;
        var to = Reef.scoringPose(a, Reef.Branch.C);
        var pts = Reef.route(a, Reef.scoringPose(a, Reef.Branch.B).getTranslation(), to);
        assertEquals(2, pts.size());
        assertEquals(2.0, pts.get(0).getTranslation().getDistance(centre(a)), 1e-6);
        assertEquals(to.getRotation(), pts.get(0).getRotation());
        assertEquals(to, pts.get(1));
    }

    @Test void everyRouteLegIsClear() {
        for (var a : Alliance.values()) {
            var targets = new ArrayList<Pose2d>();
            for (var b : Reef.Branch.values()) targets.add(Reef.scoringPose(a, b));
            for (var s : Station.Side.values()) targets.add(Station.pickupPose(a, s));
            for (double r : new double[] {1.3, 1.7, 2.5, 4.0}) for (int deg = 0; deg < 360; deg += 15) {
                var from = centre(a).plus(new Translation2d(r, Rotation2d.fromDegrees(deg)));
                for (var to : targets) {
                    var pts = Reef.route(a, from, to);
                    String ctx = a + " from r=" + r + " deg=" + deg + " to " + to;
                    assertTrue(pts.size() <= 6, ctx);
                    assertEquals(to, pts.get(pts.size() - 1), ctx);
                    if (pts.size() == 1) {
                        assertTrue(Reef.pathClear(a, from, to.getTranslation())
                                || Reef.nearestFace(a, from) == Reef.nearestFace(a, to.getTranslation()), ctx);
                        continue;
                    }
                    var prev = from;
                    for (var p : pts) {
                        assertTrue(Reef.pathClear(a, prev, p.getTranslation()), ctx + " leg to " + p);
                        prev = p.getTranslation();
                    }
                }
            }
        }
    }
}
