package frc.robot.config;

/** Every mechanism position: carriage inches of travel and wrist degrees from level (positive up, stow -90). */
public final class Setpoints {
    private Setpoints() {}

    public enum Pose {
        STOW(0, -90),
        STATION(1.25, -35),
        L1(5, -35), // PLACEHOLDER(drawing)
        L2(17.5, -35),
        L3(33.3, -35),
        // Drawn at 66 in, -53: the soft limit stops 0.75 in short, and -52 points at the same branch tip.
        L4(65.25, -52),
        ALGAE_LOW(8.8, 46),
        ALGAE_HIGH(24.6, 46),
        ALGAE_CARRY(1.25, 45),
        // Drawn at 66 in; 0.75 in lower leaves about 3.1 in over an 88.9 in rim.
        NET(65.25, 75),
        PROCESSOR(1.25, 15), // PLACEHOLDER(robot-check) marked marginal on the drawings
        CLIMB(1.25, 0);

        public final double carriageIn;
        public final double wristDeg;

        Pose(double carriageIn, double wristDeg) {
            this.carriageIn = carriageIn;
            this.wristDeg = wristDeg;
        }
    }

    public static final double CARRIAGE_TOL_IN = 0.5;
    public static final double WRIST_TOL_DEG = 1.5;
}
