package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.Auto;
import frc.robot.autos.AutoChooser;
import frc.robot.field.Reef;
import frc.robot.field.StartPose;
import frc.robot.sim.ScoreJudge.CoralScore;
import frc.robot.testutil.AlertProbe;
import frc.robot.testutil.AutoHarness;
import frc.robot.testutil.RobotFixture;
import frc.robot.testutil.SimHarness;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

@Tag("sim")
class AutoMatchTest {
    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        RobotFixture.get().sim().setCoralSensorConnected(true);
    }

    static Stream<Arguments> everyAutoOnBothAlliances() {
        return Stream.of(Auto.values())
                .flatMap(a -> Stream.of(Arguments.of(a, Alliance.Blue), Arguments.of(a, Alliance.Red)));
    }

    @ParameterizedTest
    @MethodSource("everyAutoOnBothAlliances")
    void autoScoresWhatItClaims(Auto auto, Alliance alliance) {
        var h = AutoHarness.start(auto, alliance);
        SimHarness.takeStepStats();
        SimHarness.run(15.0);
        var scored = h.sim().coralScored();
        System.out.println("AUTO " + auto + " " + alliance + ": scored " + scored.stream()
                .map(s -> s.branch() + String.format("@%.1fs", s.timeS())).toList() + "; " + SimHarness.takeStepStats());
        assertEquals(h.routine().expectedCoral(), scored.stream().map(CoralScore::branch).toList(), "scored " + scored);
        assertTrue(scored.stream().allMatch(s -> s.alliance() == alliance), "scored on the wrong reef: " + scored);
        assertTrue(scored.stream().allMatch(s -> s.timeS() <= 15.0), "late: " + scored);
        assertTrue(h.keepOutNeverViolated(), "keep-out broken at " + h.firstViolation());
        assertTrue(h.chooser().poseSetForSelected());
        if (auto == Auto.CENTRE_H_ALGAE) {
            assertEquals(1, h.sim().algaeInNet());
        }
    }

    @Test
    void expectedCoralMatchesTheDesign() {
        var chooser = RobotFixture.get().autoChooser();
        var expected = java.util.Map.of(
                Auto.IDLE, List.<Reef.Branch>of(),
                Auto.LEAVE, List.<Reef.Branch>of(),
                Auto.ONE_RIGHT, List.of(Reef.Branch.E),
                Auto.ONE_CENTRE, List.of(Reef.Branch.H),
                Auto.ONE_LEFT, List.of(Reef.Branch.J),
                Auto.THREE_RIGHT_ECD, List.of(Reef.Branch.E, Reef.Branch.C, Reef.Branch.D),
                Auto.THREE_LEFT_JLK, List.of(Reef.Branch.J, Reef.Branch.L, Reef.Branch.K),
                Auto.CENTRE_H_ALGAE, List.of(Reef.Branch.H));
        for (var a : Auto.values()) {
            assertEquals(expected.get(a), chooser.routine(Alliance.Blue, a).expectedCoral(), a.name());
        }
    }

    @Test
    void labelsAreTheDashboardNames() {
        assertEquals(
                List.of("Idle", "Leave", "1 Right", "1 Centre", "1 Left", "3 Right: E, C, D", "3 Left: J, L, K",
                        "Centre: H and algae"),
                AutoChooser.PROGRAMS.stream().map(p -> p.label()).toList());
    }

    @Test
    void startPoseMatchesTheRoutine() {
        var r = RobotFixture.reset(StartPose.CENTRE.pose(Alliance.Blue), false);
        var chooser = r.autoChooser();
        SimHarness.disable();
        for (var a : Auto.values()) {
            chooser.select(a);
            chooser.update();
            var start = chooser.startPose();
            if (start.isEmpty()) {
                continue;
            }
            chooser.setPoseToStart();
            SimHarness.run(0.1);
            assertEquals(0, r.drive().pose().getTranslation().getDistance(start.get().getTranslation()), 0.02, a.name());
            assertEquals(0, r.drive().pose().getRotation().minus(start.get().getRotation()).getDegrees(), 1, a.name());
            assertTrue(chooser.poseSetForSelected(), a.name());
        }
    }

    @Test
    void unknownAllianceRunsNothing() {
        var chooser = RobotFixture.reset(StartPose.CENTRE.pose(Alliance.Blue), true).autoChooser();
        RobotFixture.setAlliance(AllianceStationID.Unknown);
        chooser.select(Auto.ONE_CENTRE);
        chooser.update();
        assertTrue(chooser.selectedCommand().isEmpty());
        assertTrue(chooser.startAuto().isEmpty());
        assertTrue(AlertProbe.active("No alliance: auto not run"));
    }

    @Test
    void allianceSwitchAfterBuildUsesTheRightSide() {
        var r = RobotFixture.reset(StartPose.CENTRE.pose(Alliance.Blue), false);
        var chooser = r.autoChooser();
        SimHarness.disable();
        chooser.select(Auto.ONE_CENTRE);
        chooser.update(); // built for both alliances while blue
        RobotFixture.setAlliance(AllianceStationID.Red1);
        r.sim().setTruePose(StartPose.CENTRE.pose(Alliance.Red));
        SimHarness.run(0.2);
        assertFalse(chooser.poseSetForSelected(), "the alliance changed: the pose must be set again");
        chooser.setPoseToStart();
        r.sim().preloadCoral();
        SimHarness.run(0.2);
        SimHarness.enableAutonomous();
        CommandScheduler.getInstance().schedule(chooser.startAuto().orElseThrow());
        SimHarness.run(8.0);
        var scored = r.sim().coralScored();
        assertEquals(1, scored.size(), "scored " + scored);
        assertEquals(Alliance.Red, scored.get(0).alliance());
        assertEquals(Reef.Branch.H, scored.get(0).branch());
    }

    @Test
    void anAutoStartedWithoutThePoseButtonAlertsAndUsesItsStartPose() {
        var r = RobotFixture.reset(StartPose.CENTRE.pose(Alliance.Blue), true);
        var chooser = r.autoChooser();
        SimHarness.disable();
        chooser.select(Auto.ONE_CENTRE);
        chooser.update();
        r.drive().resetPose(new edu.wpi.first.math.geometry.Pose2d()); // odometry left at the origin
        SimHarness.run(0.1);
        assertFalse(chooser.poseSetForSelected());
        SimHarness.enableAutonomous();
        var command = chooser.startAuto();
        assertTrue(command.isPresent());
        assertTrue(AlertProbe.active("Auto started without its start pose set"));
        SimHarness.run(0.1);
        var start = StartPose.CENTRE.pose(Alliance.Blue);
        assertEquals(0, r.drive().pose().getTranslation().getDistance(start.getTranslation()), 0.05,
                "the estimate should be put on the auto's start pose, not left at the origin");
    }

    @Test
    void thePoseButtonDoesNothingWhileEnabled() {
        var r = RobotFixture.reset(StartPose.CENTRE.pose(Alliance.Blue), false);
        var chooser = r.autoChooser();
        // The robot is shared between tests: first make sure the flag belongs to some other auto.
        SimHarness.disable();
        chooser.select(Auto.IDLE);
        chooser.setPoseToStart();
        SimHarness.enable();
        chooser.select(Auto.ONE_LEFT);
        chooser.update();
        var before = r.drive().pose();
        chooser.setPoseToStart(); // enabled in teleop: a stray dashboard click must not teleport odometry
        SimHarness.run(0.1);
        assertEquals(0, r.drive().pose().getTranslation().getDistance(before.getTranslation()), 0.05);
        assertFalse(chooser.poseSetForSelected());
    }

    @Test
    void poseFlagClearsWhenSelectionChanges() {
        var chooser = RobotFixture.reset(StartPose.CENTRE.pose(Alliance.Blue), false).autoChooser();
        SimHarness.disable();
        chooser.select(Auto.ONE_LEFT);
        chooser.update();
        chooser.setPoseToStart();
        assertTrue(chooser.poseSetForSelected());
        chooser.select(Auto.ONE_RIGHT);
        chooser.update();
        assertFalse(chooser.poseSetForSelected());
    }

    /**
     * With the coral sensor dead no leg can end on its sensor, so every leg must end on its timeout:
     * the routine is then bounded by the sum of its leg timeouts (5 legs of 6 s) instead of hanging.
     * Scoring legs run blind rather than skip, so the preload still scores.
     */
    @Test
    void deadCoralSensorEndsLegsOnTimeout() {
        var h = AutoHarness.start(Auto.THREE_RIGHT_ECD, Alliance.Blue, () -> RobotFixture.get().sim().setCoralSensorConnected(false));
        double boundS = 5 * 6.0 + 1.0;
        double elapsed = 0;
        while (h.commandStillRunningALeg() && elapsed < boundS) {
            SimHarness.run(0.5);
            elapsed += 0.5;
        }
        assertFalse(h.commandStillRunningALeg(), "still running after " + boundS + " s: a leg is waiting on a sensor that will never answer");
        var scored = h.sim().coralScored();
        System.out.println("DEAD SENSOR: routine ended after " + elapsed + " s, scored " + scored);
        assertTrue(scored.stream().anyMatch(sc -> sc.branch() == Reef.Branch.E && sc.timeS() <= 15.0),
                "the preload should still score inside the auto period, scored " + scored);
        assertTrue(h.keepOutNeverViolated(), "keep-out broken at " + h.firstViolation());
    }
}
