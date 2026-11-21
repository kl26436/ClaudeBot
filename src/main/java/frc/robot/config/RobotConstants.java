package frc.robot.config;

import edu.wpi.first.wpilibj.RobotBase;

/** Robot-wide constants. Every dimension and ratio lives here. */
public final class RobotConstants {
    private RobotConstants() {}

    /** Run mode chosen at startup. */
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    /** What a non-real run does: {@link Mode#SIM} to simulate, {@link Mode#REPLAY} to replay a log. */
    private static final Mode SIM_MODE = Mode.SIM;

    public static final int TEAM_NUMBER = 4206;

    /** Phoenix Pro licence present. When false, no code path may request FOC, fused sensors or time-synced signals. */
    public static final boolean PRO_LICENSED = true;

    public static Mode currentMode() {
        return RobotBase.isReal() ? Mode.REAL : SIM_MODE;
    }
}
