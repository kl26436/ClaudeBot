package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the wrist. Angles are degrees from level, positive up; stow is -90. */
public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public boolean connected;
        /** True while the absolute encoder's signal is reaching the roboRIO. */
        public boolean encoderConnected;
        /** Angle the motor controller is closing its loop on: its own rotor, seeded from the encoder. */
        public double wristDeg;
        /** Angle straight from the absolute encoder. */
        public double absoluteDeg;
        public double velocityDegPerS;
        public double appliedVolts;
        public double statorAmps;
        /** True on the loop the motor controller reports it has rebooted since the last loop. */
        public boolean resetOccurred;
    }

    default void updateInputs(WristIOInputs in) {}

    /** Tell the motor controller the angle the wrist is really at. */
    default void seed(double deg) {}

    /** Closed-loop (Motion Magic) move to an angle. */
    default void setTargetDeg(double deg) {}

    default void stop() {}
}
