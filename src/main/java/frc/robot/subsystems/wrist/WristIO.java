package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the wrist. Angles are degrees from level, positive up; stow is -90. */
public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public boolean connected;
        /** True while the absolute encoder's PWM signal is reaching the CANdi. */
        public boolean encoderConnected;
        /** Angle the motor controller is closing its loop on. */
        public double wristDeg;
        /** Angle straight from the absolute encoder. */
        public double absoluteDeg;
        public double velocityDegPerS;
        public double appliedVolts;
        public double statorAmps;
    }

    default void updateInputs(WristIOInputs in) {}

    /** Closed-loop (Motion Magic) move to an angle. */
    default void setTargetDeg(double deg) {}

    default void stop() {}
}
