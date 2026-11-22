package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.Setpoints.Pose;
import frc.robot.field.Barge;
import frc.robot.field.Reef;
import frc.robot.sim.ScoreJudge;
import frc.robot.subsystems.Superstructure.Wanted;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.api.Test;

class ScoreJudgeTest {
    @ParameterizedTest @EnumSource(Alliance.class)
    void everyLevelScoresAtItsSetpoint(Alliance a) {
        Pose[] poses = {Pose.L1, Pose.L2, Pose.L3, Pose.L4};
        Wanted[] levels = {Wanted.L1, Wanted.L2, Wanted.L3, Wanted.L4};
        for (var b : Reef.Branch.values()) {
            for (int i = 0; i < 4; i++) {
                var s = ScoreJudge.judgeCoral(a, Reef.scoringPose(a, b), poses[i].carriageIn, poses[i].wristDeg, 3.0);
                assertTrue(s.isPresent(), b + " " + levels[i]);
                assertEquals(b, s.get().branch());
                assertEquals(levels[i], s.get().level());
                assertEquals(a, s.get().alliance());
                assertEquals(3.0, s.get().timeS());
            }
        }
    }

    @Test void twoInchesOffTheBranchMisses() {
        var p = Reef.scoringPose(Alliance.Blue, Reef.Branch.C).transformBy(new Transform2d(0, Units.inchesToMeters(2), Rotation2d.kZero));
        assertTrue(ScoreJudge.judgeCoral(Alliance.Blue, p, Pose.L4.carriageIn, Pose.L4.wristDeg, 0).isEmpty());
    }

    @Test void turnedThreeDegreesMisses() {
        var p = Reef.scoringPose(Alliance.Blue, Reef.Branch.C).transformBy(new Transform2d(0, 0, Rotation2d.fromDegrees(3)));
        assertTrue(ScoreJudge.judgeCoral(Alliance.Blue, p, Pose.L4.carriageIn, Pose.L4.wristDeg, 0).isEmpty());
    }

    @Test void theWrongWristAngleMisses() {
        var p = Reef.scoringPose(Alliance.Blue, Reef.Branch.C);
        assertTrue(ScoreJudge.judgeCoral(Alliance.Blue, p, Pose.L4.carriageIn, Pose.L4.wristDeg + 5, 0).isEmpty());
        assertTrue(ScoreJudge.judgeCoral(Alliance.Blue, p, Pose.L3.carriageIn + 4, Pose.L3.wristDeg, 0).isEmpty());
    }

    @Test void stowedAtTheReefScoresNothing() {
        var p = Reef.scoringPose(Alliance.Blue, Reef.Branch.C);
        assertTrue(ScoreJudge.judgeCoral(Alliance.Blue, p, Pose.STOW.carriageIn, Pose.STOW.wristDeg, 0).isEmpty());
    }

    @ParameterizedTest @EnumSource(Alliance.class)
    void netScoresFromTheNetPose(Alliance a) {
        assertTrue(ScoreJudge.judgeNet(a, Barge.netPose(a), Pose.NET.carriageIn, Pose.NET.wristDeg));
    }

    @Test void netNeedsTheNetPoseAndTheLine() {
        var at = Barge.netPose(Alliance.Blue);
        assertFalse(ScoreJudge.judgeNet(Alliance.Blue, at, Pose.L4.carriageIn, Pose.L4.wristDeg), "wrist not at net");
        var back = at.transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero));
        assertFalse(ScoreJudge.judgeNet(Alliance.Blue, back, Pose.NET.carriageIn, Pose.NET.wristDeg), "0.5 m short of the line");
        var turned = new Pose2d(at.getTranslation(), at.getRotation().plus(Rotation2d.k180deg));
        assertFalse(ScoreJudge.judgeNet(Alliance.Blue, turned, Pose.NET.carriageIn, Pose.NET.wristDeg), "facing away");
        assertFalse(ScoreJudge.judgeNet(Alliance.Red, at, Pose.NET.carriageIn, Pose.NET.wristDeg), "blue's net is not red's");
    }
}
