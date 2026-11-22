package frc.robot.rules;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.testutil.PoseAsserts.assertPoseEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.field.AllianceFlip;
import frc.robot.field.Reef;
import frc.robot.field.StartPose;
import frc.robot.field.Station;
import org.junit.jupiter.api.Test;

class ReefTest {
    @Test void branchPosesSitSquareAtBumperDistance() {
        for (var a : Alliance.values()) for (var b : Reef.Branch.values()) {
            var tag = Reef.tagPose(a, b.face()); var robot = Reef.scoringPose(a, b);
            var rel = robot.relativeTo(tag);
            assertEquals(Units.inchesToMeters(17.4), rel.getX(), 1e-6);
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
            assertEquals(Units.inchesToMeters(17.9), rel.getX(), 1e-6);
            assertEquals(0, rel.getY(), 1e-6);
            assertEquals(f % 2 == 0, Reef.algaeIsHigh(f));
        }
        assertEquals(3, Reef.nearestFace(Alliance.Blue, Reef.tagPose(Alliance.Blue, 3).getTranslation()));
    }

    @Test void startPosesMirrorForRed() {
        for (var s : StartPose.values()) assertPoseEquals(AllianceFlip.rotate(s.pose(Alliance.Blue)), s.pose(Alliance.Red), 1e-9);
        assertEquals(180, Math.abs(StartPose.LEFT.pose(Alliance.Blue).getRotation().getDegrees()), 1e-9);
    }
}
