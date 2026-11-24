package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.DeviceCheck;
import frc.robot.sim.SimulatedRobot;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Main robot class. AdvantageKit logging is configured per {@link RobotConstants#currentMode()}. */
public class Robot extends LoggedRobot {
    private static final String SIM_LOG_DIRECTORY = "logs";
    private static final String USB_LOG_DIRECTORY = "/U/logs";

    private static final double SIM_PERIOD_S = 0.005;
    /** A longer gap (a breakpoint, a stall) is not simulated as one huge step. */
    private static final double MAX_SIM_DT_S = 0.05;

    private RobotContainer robotContainer;
    private Notifier simNotifier;

    public Robot() {
        Logger.recordMetadata("ProjectName", "ClaudeBot");
        Logger.recordMetadata("TeamNumber", Integer.toString(RobotConstants.TEAM_NUMBER));
        Logger.recordMetadata("RunMode", RobotConstants.currentMode().name());

        switch (RobotConstants.currentMode()) {
            case REAL -> {
                // USB stick (FAT32, plugged into a roboRIO port) and NetworkTables.
                Logger.addDataReceiver(new WPILOGWriter(USB_LOG_DIRECTORY));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter(SIM_LOG_DIRECTORY));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                // Run as fast as possible; read the chosen log and write a "_sim" copy with replayed outputs.
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.start();

        robotContainer = new RobotContainer();

        List<String> missing = DeviceCheck.missing();
        Logger.recordOutput("DeviceCheck/Missing", missing.toArray(new String[0]));
        for (String name : missing) {
            new Alert("Not answering: " + name, AlertType.kError).set(true);
        }

        if (robotContainer.sim() != null) {
            startSimulationNotifier(robotContainer.sim());
        }
    }

    /**
     * Step the simulated world every 5 ms of real time. Phoenix's simulated controllers close their
     * loops at 1 kHz; physics that moves only every 20 ms looks to them like a sawtooth error.
     */
    private void startSimulationNotifier(SimulatedRobot sim) {
        double[] last = {Timer.getFPGATimestamp()};
        simNotifier = new Notifier(() -> {
            double now = Timer.getFPGATimestamp();
            double dt = Math.min(now - last[0], MAX_SIM_DT_S);
            last[0] = now;
            if (dt > 0) {
                sim.simulate(dt);
            }
        });
        simNotifier.setName("Simulation");
        simNotifier.startPeriodic(SIM_PERIOD_S);
    }

    private Command autoCommand;

    @Override
    public void disabledPeriodic() {
        robotContainer.autoChooser().update();
    }

    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.autoChooser().startAuto().orElse(null);
        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
            autoCommand = null;
        }
    }

    @Override
    public void robotPeriodic() {
        // Give the main thread real-time priority while the scheduler runs, then restore it.
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10);
    }
}
