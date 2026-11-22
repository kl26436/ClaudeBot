package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.DeviceCheck;
import frc.robot.commands.ScoreCommands;
import frc.robot.field.Reef;
import frc.robot.field.Station;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.testutil.SimHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class TeleopCycleTest {
    private RobotContainer r;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        r = new RobotContainer();
        SimHarness.enable();
        SimHarness.run(0.06);
        SimHarness.register(() -> r.sim().simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14);
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        r.close();
    }

    private void startAt(Pose2d pose, boolean coral) {
        r.sim().setTruePose(pose);
        SimHarness.run(0.1);
        r.drive().resetPose(pose);
        if (coral) {
            r.sim().preloadCoral();
        }
        SimHarness.run(0.2);
        assertEquals(coral, r.superstructure().hasCoral(), "precondition: coral on board is " + coral);
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
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
        var start = Reef.scoringPose(Alliance.Blue, Reef.Branch.H)
                .transformBy(new edu.wpi.first.math.geometry.Transform2d(-1.0, 0, edu.wpi.first.math.geometry.Rotation2d.kZero));
        startAt(start, true);
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
        assertEquals(h / 3, raised[0].getZ(), 0.02, "stage A");
        assertEquals(2 * h / 3, raised[1].getZ(), 0.02, "stage B");
        assertEquals(h, raised[2].getZ(), 0.02, "carriage");
        assertEquals(Math.toRadians(35), raised[3].getRotation().getY(), Math.toRadians(2), "wrist at -35 is pitched 35 nose-up");
    }

    @Test
    void deviceCheckIsCleanInSimulation() {
        assertTrue(DeviceCheck.missing().isEmpty(), "missing: " + DeviceCheck.missing());
    }
}
