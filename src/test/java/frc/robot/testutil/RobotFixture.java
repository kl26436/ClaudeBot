package frc.robot.testutil;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.Wanted;
import java.util.Optional;

/**
 * One whole simulated robot per test JVM (simTest forks one per class), reset between tests rather
 * than rebuilt. Phoenix's simulator slows down as devices are created and closed, so a class that
 * built a fresh robot for every test had its later tests run on a struggling simulator.
 */
public final class RobotFixture {
    private RobotFixture() {}

    /** Longest the mechanism may take to get back to stow during a reset. */
    private static final double STOW_TIMEOUT_S = 3.0;

    private static RobotContainer robot;

    /** The robot, built and running on the first call. */
    public static RobotContainer get() {
        if (robot == null) {
            SimHarness.reset();
            setAlliance(AllianceStationID.Blue1);
            robot = new RobotContainer();
            SimHarness.enable();
            SimHarness.run(0.06);
            RobotContainer r = robot;
            SimHarness.register(() -> r.sim().simulate(SimHarness.lastDtSeconds()));
            SimHarness.run(0.14);
        }
        return robot;
    }

    public static void setAlliance(AllianceStationID station) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
    }

    /**
     * Cancel everything, stow, clear the field, then put the robot at {@code start} (odometry told) with
     * or without a coral. Leaves the robot enabled in teleop on the blue alliance.
     */
    public static RobotContainer reset(Pose2d start, boolean coral) {
        RobotContainer r = get();
        CommandScheduler.getInstance().cancelAll();
        setAlliance(AllianceStationID.Blue1);
        SimHarness.enable();
        r.sim().resetField();
        r.superstructure().requestRelease(false);
        r.superstructure().setWanted(Wanted.STOW);
        r.vision().setTargetFace(Optional.empty());
        r.drive().stopDriving();
        int ticks = 0;
        while (!r.superstructure().atPose() && ticks++ < STOW_TIMEOUT_S / SimHarness.DT) {
            SimHarness.run(SimHarness.DT);
        }
        assertTrue(r.superstructure().atPose(), "reset: the mechanism did not get back to stow");
        SimHarness.run(0.3); // let the wheels stop before the teleport
        r.sim().setTruePose(start);
        SimHarness.run(0.1); // the Pigeon publishes its new yaw before odometry is told
        r.drive().resetPose(start);
        if (coral) {
            r.sim().preloadCoral();
        }
        SimHarness.run(0.2);
        assertEquals(coral, r.superstructure().hasCoral(), "reset: coral on board should be " + coral);
        return r;
    }
}
