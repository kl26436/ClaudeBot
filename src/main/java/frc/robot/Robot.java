package frc.robot;

import edu.wpi.first.wpilibj.Threads;
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

    private RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", "RobotA");
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
    }

    @Override
    public void robotPeriodic() {
        // Give the main thread real-time priority while the scheduler runs, then restore it.
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10);
    }
}
