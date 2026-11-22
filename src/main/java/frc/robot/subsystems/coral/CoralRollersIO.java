package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the coral rollers and the CANrange that sees a seated coral. */
public interface CoralRollersIO {
    @AutoLog
    public static class CoralRollersIOInputs {
        public boolean connected;
        /** True while the CANrange is answering on the bus. */
        public boolean rangeConnected;
        /** Roller speed at the output shaft, positive inward. */
        public double velocityRps;
        public double appliedVolts;
        public double statorAmps;
        /** Distance the CANrange reads, in metres. */
        public double rangeM;
    }

    default void updateInputs(CoralRollersIOInputs in) {}

    /** Open-loop voltage; positive pulls the piece in. */
    default void setVolts(double volts) {}
}
