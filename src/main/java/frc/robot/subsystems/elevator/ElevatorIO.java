package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the elevator. Positions are inches of carriage travel above the home switch. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected;
        public boolean followerConnected;
        /** True while the carriage is on the bottom home switch. */
        public boolean homeSwitch;

        public double carriageIn;
        public double velocityInPerS;
        public double appliedVolts;
        public double statorAmps;
        public double supplyAmps;
        /** True on the loop the motor controller reports it has rebooted since the last loop. */
        public boolean resetOccurred;
    }

    default void updateInputs(ElevatorIOInputs in) {}

    /** Closed-loop (Motion Magic) move to a carriage height. */
    default void setTargetIn(double carriageIn) {}

    /** Open-loop voltage, used only while homing. */
    default void setVolts(double volts) {}

    /** Declare the present carriage position to be zero. */
    default void zero() {}

    /** Keep the position the motor controller already has (a zero from before a code restart). */
    default void keepZero() {}

    default void stop() {}
}
