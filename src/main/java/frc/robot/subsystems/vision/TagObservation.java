package frc.robot.subsystems.vision;

/** One AprilTag sighting. {@code txDeg} is positive to the right, as the Limelight reports it. */
public record TagObservation(int tagId, double txDeg, double heightPx) {}
