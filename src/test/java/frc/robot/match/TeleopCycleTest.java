package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.DeviceCheck;
import frc.robot.commands.ScoreCommands;
import frc.robot.field.Reef;
import frc.robot.field.Station;
import frc.robot.sim.ScoreJudge.CoralScore;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.testutil.RobotFixture;
import frc.robot.testutil.SimHarness;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class TeleopCycleTest {
    private RobotContainer r;

    /** Open carpet, clear of the reef, where each test starts unless it places the robot itself. */
    private static final Pose2d OPEN_FIELD = new Pose2d(2.5, 6.5, edu.wpi.first.math.geometry.Rotation2d.kZero);

    @BeforeEach
    void setUp() {
        r = RobotFixture.reset(OPEN_FIELD, false);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void startAt(Pose2d pose, boolean coral) {
        RobotFixture.reset(pose, coral);
    }

    private static void schedule(Command c) {
        CommandScheduler.getInstance().schedule(c);
    }

    @Test
    void stationToL4Cycle() {
        startAt(Station.pickupPose(Alliance.Blue, Station.Side.RIGHT), false);
        schedule(ScoreCommands.stationPickup(r, () -> Station.Side.RIGHT, 3.0, true));
        SimHarness.run(2.0);
        assertTrue(r.superstructure().hasCoral());
        schedule(ScoreCommands.alignAndScore(r, () -> Reef.Branch.C, () -> Wanted.L4, 3.0, true));
        SimHarness.run(6.0);
        assertFalse(r.superstructure().hasCoral());
        assertEquals(Wanted.STOW, r.superstructure().wanted());
        // The score judge saw it land on the named branch.
        assertEquals(List.of(Reef.Branch.C), r.sim().coralScored().stream().map(CoralScore::branch).toList());
        assertEquals(Wanted.L4, r.sim().coralScored().get(0).level());
    }

    @Test
    void releaseAwayFromTheReefDoesNotScore() {
        startAt(new Pose2d(2.5, 6.5, edu.wpi.first.math.geometry.Rotation2d.kZero), true);
        r.superstructure().setWanted(Wanted.L4);
        SimHarness.run(2.5);
        r.superstructure().requestRelease(true);
        SimHarness.run(0.5);
        assertFalse(r.superstructure().hasCoral(), "the coral left");
        assertTrue(r.sim().coralScored().isEmpty());
    }

    @Test
    void algaeComesOffTheFace() {
        var face = 3;
        startAt(Reef.algaePose(Alliance.Blue, face)
                .transformBy(new edu.wpi.first.math.geometry.Transform2d(-1.0, 0, edu.wpi.first.math.geometry.Rotation2d.kZero)), false);
        assertTrue(r.sim().reefAlgaePresent(Alliance.Blue, face));
        schedule(ScoreCommands.algaeFromReef(r, () -> face));
        SimHarness.run(5);
        assertTrue(r.superstructure().hasAlgae());
        assertFalse(r.sim().reefAlgaePresent(Alliance.Blue, face));
        assertTrue(r.sim().reefAlgaePresent(Alliance.Blue, 0), "other faces keep theirs");
    }

    @Test
    void netScoreCountsAnAlgae() {
        startAt(frc.robot.field.Barge.netPose(Alliance.Blue), false);
        r.sim().giveAlgae();
        SimHarness.run(0.2);
        schedule(ScoreCommands.netScore(r));
        SimHarness.run(3.0);
        assertFalse(r.superstructure().hasAlgae());
        assertEquals(1, r.sim().algaeInNet());
    }

    @Test
    void releasingTheButtonStowsAndClearsTheTarget() {
        startAt(Reef.scoringPose(Alliance.Blue, Reef.Branch.H)
                .transformBy(new edu.wpi.first.math.geometry.Transform2d(-1.0, 0, edu.wpi.first.math.geometry.Rotation2d.kZero)), true);
        var score = ScoreCommands.alignAndScore(r, () -> Reef.Branch.H, () -> Wanted.L4, 3.0, true);
        schedule(score);
        SimHarness.run(0.8);
        assertEquals(Wanted.L4, r.superstructure().wanted(), "raises once within 48 in");
        score.cancel();
        SimHarness.run(0.1);
        assertEquals(Wanted.STOW, r.superstructure().wanted());
        assertTrue(r.vision().lastFix().isEmpty(), "the vision target face is cleared");
        assertTrue(r.superstructure().hasCoral(), "cancelled before the release");
    }

    @Test
    void scoringWithNoAllianceDoesNothing() {
        var start = Reef.scoringPose(Alliance.Blue, Reef.Branch.H)
                .transformBy(new edu.wpi.first.math.geometry.Transform2d(-1.0, 0, edu.wpi.first.math.geometry.Rotation2d.kZero));
        startAt(start, true);
        RobotFixture.setAlliance(AllianceStationID.Unknown);
        var score = ScoreCommands.alignAndScore(r, () -> Reef.Branch.H, () -> Wanted.L4, 3.0, true);
        schedule(score);
        SimHarness.run(0.5);
        assertFalse(score.isScheduled(), "ends at once without an alliance");
        assertEquals(Wanted.STOW, r.superstructure().wanted());
        assertTrue(r.superstructure().hasCoral());
    }

    @Test
    void componentPosesFollowTheMechanism() {
        var stowed = r.sim().componentPoses();
        assertEquals(4, stowed.length);
        assertEquals(0, stowed[2].getZ(), 0.01, "carriage at rest");
        assertEquals(edu.wpi.first.math.util.Units.inchesToMeters(24.5), stowed[3].getZ(), 0.01, "wrist pivot at rest height");
        r.superstructure().setWanted(Wanted.L3);
        SimHarness.run(2.5);
        var raised = r.sim().componentPoses();
        double h = edu.wpi.first.math.util.Units.inchesToMeters(33.3);
        String state = " (carriage " + raised[2].getZ() / 0.0254 + " in, wrist pitch "
                + Math.toDegrees(raised[3].getRotation().getY()) + " deg)";
        assertEquals(h / 3, raised[0].getZ(), 0.02, "stage A" + state);
        assertEquals(2 * h / 3, raised[1].getZ(), 0.02, "stage B");
        assertEquals(h, raised[2].getZ(), 0.02, "carriage");
        assertEquals(Math.toRadians(35), raised[3].getRotation().getY(), Math.toRadians(2), "wrist at -35 is pitched 35 nose-up");
    }

    @Test
    void deviceCheckIsCleanInSimulation() {
        assertTrue(DeviceCheck.missing().isEmpty(), "missing: " + DeviceCheck.missing());
    }
}
