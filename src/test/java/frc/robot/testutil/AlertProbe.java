package frc.robot.testutil;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;

/** Reads the WPILib alert lists that the dashboard shows, so tests can assert an alert is raised. */
public final class AlertProbe {
    private AlertProbe() {}

    /** True when an alert with exactly this text is active at any level (error, warning or info). */
    public static boolean active(String text) {
        SmartDashboard.updateValues(); // alerts are published when the dashboard is refreshed
        for (String level : new String[] {"errors", "warnings", "infos"}) {
            String[] alerts = NetworkTableInstance.getDefault()
                    .getStringArrayTopic("/SmartDashboard/Alerts/" + level)
                    .subscribe(new String[0])
                    .get();
            if (Arrays.asList(alerts).contains(text)) {
                return true;
            }
        }
        return false;
    }
}
