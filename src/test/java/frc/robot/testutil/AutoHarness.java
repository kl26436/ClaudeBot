package frc.robot.testutil;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.autos.AutoChooser;
import frc.robot.autos.AutoRoutine;
import frc.robot.config.KeepOut;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.RobotConstants.WristK;
import frc.robot.field.StartPose;
import frc.robot.sim.SimulatedRobot;
import java.util.Optional;

/**
 * Runs one auto the way a match does: alliance set, auto selected and built while disabled, the
 * start-pose button pressed, a coral preloaded, then autonomous enabled.
 */
public final class AutoHarness {
    private static boolean monitorRegistered = false;
    private static volatile boolean keepOutViolated = false;
    private static volatile String firstViolation = "";
    private static volatile double maxWristGapDeg = 0;
    private static RobotContainer watched;

    private final RobotContainer robot;
    private final AutoRoutine routine;
    private final Command command;

    private AutoHarness(RobotContainer robot, AutoRoutine routine, Command command) {
        this.robot = robot;
        this.routine = routine;
        this.command = command;
    }

    public static AutoHarness start(Auto auto, Alliance alliance) {
        return start(auto, alliance, () -> {});
    }

    /** As {@link #start(Auto, Alliance)}, running {@code beforeEnable} just before autonomous starts. */
    public static AutoHarness start(Auto auto, Alliance alliance, Runnable beforeEnable) {
        RobotContainer r = RobotFixture.get();
        AutoChooser chooser = r.autoChooser();
        chooser.select(auto);
        RobotFixture.setAlliance(alliance == Alliance.Blue ? AllianceStationID.Blue1 : AllianceStationID.Red1);
        AutoRoutine routine = chooser.routine(alliance);
        Pose2d start = routine.startPose().orElse(StartPose.CENTRE.pose(alliance));
        RobotFixture.reset(start, false);
        RobotFixture.setAlliance(alliance == Alliance.Blue ? AllianceStationID.Blue1 : AllianceStationID.Red1);

        SimHarness.disable();
        chooser.select(auto);
        chooser.update();
        SimHarness.run(0.1);
        chooser.setPoseToStart();
        r.sim().preloadCoral();
        SimHarness.run(0.2);
        beforeEnable.run();
        watchKeepOut(r.sim());

        SimHarness.enableAutonomous();
        Optional<Command> command = chooser.startAuto();
        assertTrue(command.isPresent(), "the auto should start with a known alliance");
        CommandScheduler.getInstance().schedule(command.get());
        return new AutoHarness(r, routine, command.get());
    }

    /** Check every physics step that the real mechanism is outside every keep-out. */
    private static void watchKeepOut(SimulatedRobot sim) {
        keepOutViolated = false;
        firstViolation = "";
        maxWristGapDeg = 0;
        if (!monitorRegistered) {
            monitorRegistered = true;
            SimHarness.register(() -> {
                double h = sim.trueCarriageIn();
                double w = sim.trueWristDeg();
                maxWristGapDeg = Math.max(maxWristGapDeg, Math.abs(RobotFixture.get().wrist().wristDeg() - w));
                // Readings a little past a soft travel limit are judged at the limit, as the planner does.
                boolean legal = KeepOut.isLegal(
                                MathUtil.clamp(h, 0, ElevatorK.SOFT_MAX_IN), MathUtil.clamp(w, WristK.MIN_DEG, WristK.MAX_DEG))
                        && h >= -0.5 && h <= ElevatorK.SOFT_MAX_IN + 0.5
                        && w >= WristK.MIN_DEG - 1.5 && w <= WristK.MAX_DEG + 1.5;
                if (!legal && !keepOutViolated) {
                    keepOutViolated = true;
                    firstViolation = "h=" + h + " w=" + w + " (reading h=" + RobotFixture.get().elevator().carriageIn() + " w=" + RobotFixture.get().wrist().wristDeg() + ", wanted " + RobotFixture.get().superstructure().wanted() + ")";
                }
            });
        }
    }

    public AutoRoutine routine() {
        return routine;
    }

    public SimulatedRobot sim() {
        return robot.sim();
    }

    public AutoChooser chooser() {
        return robot.autoChooser();
    }

    public RobotContainer robot() {
        return robot;
    }

    public boolean keepOutNeverViolated() {
        return !keepOutViolated;
    }

    public String firstViolation() {
        return firstViolation + String.format("; largest wrist reading-to-truth gap %.2f deg", maxWristGapDeg);
    }

    /** True while the auto's command is still running one of its legs. */
    public boolean commandStillRunningALeg() {
        return command.isScheduled();
    }
}
