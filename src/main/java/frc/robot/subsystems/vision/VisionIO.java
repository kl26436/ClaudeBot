package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the intake camera: the one tag it is looking at, if any. */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected;
        public boolean hasTarget;
        public int tagId;
        /** Horizontal angle to the tag centre, positive to the right. */
        public double txDeg;
        /** Height of the tag in the image, in pixels. */
        public double heightPx;
    }

    default void updateInputs(VisionIOInputs in) {}
}
