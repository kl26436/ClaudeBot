package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the two-roller algae claw and the CANrange that sees a held algae. */
public interface AlgaeClawIO {
    @AutoLog
    public static class AlgaeClawIOInputs {
        /** True only while both claw motors are answering. */
        public boolean connected;
        /** True while the CANrange is answering on the bus. */
        public boolean rangeConnected;
        /** Front roller speed at its output shaft, positive inward. */
        public double velocityRps;
        public double appliedVolts;
        /** The larger stator current of the two motors. */
        public double statorAmps;
        /** Distance the CANrange reads, in metres. */
        public double rangeM;
    }

    default void updateInputs(AlgaeClawIOInputs in) {}

    /** Open-loop voltage to both rollers; positive pulls the piece in. */
    default void setVolts(double volts) {}
}
