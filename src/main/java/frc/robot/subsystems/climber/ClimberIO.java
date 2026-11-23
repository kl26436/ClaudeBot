package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for the climber winch, its pawls and the funnel latch. The fork angle is in
 * degrees: 90 stowed (straight up), 0 level and pointing back. It is the angle the rope sets, worked
 * from the motor; nothing measures the fork itself.
 */
public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean connected;
        public boolean followerConnected;
        public double forkDeg = 90;
        public double velocityDegPerS;
        public double appliedVolts;
        public double statorAmps;
        /** True on the loop the lead motor controller reports it has rebooted since the last loop. */
        public boolean resetOccurred;
    }

    default void updateInputs(ClimberIOInputs in) {}

    /** Closed-loop (Motion Magic) winch move to the rope length that puts the fork at this angle. */
    default void setTargetDeg(double forkDeg) {}

    default void stop() {}

    /** Declare the fork to be stowed where it is. */
    default void zeroStowed() {}

    /** Hold the pawls off the ratchet wheels, or let their springs put them back. */
    default void setPawlsLifted(boolean lifted) {}

    /** Release the funnel tray, or hold the latch closed. Nothing on the robot can close an open tray. */
    default void setFunnelOpen(boolean open) {}
}
