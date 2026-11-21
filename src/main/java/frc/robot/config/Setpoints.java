package frc.robot.config;

/** Every mechanism position: carriage inches of travel and wrist degrees from level (positive up, stow -90). */
public final class Setpoints {
    private Setpoints() {}

    public enum Pose {
        STOW(0, -90),
        STATION(1.75, -30), // PLACEHOLDER(drawing) PLACEHOLDER(funnel-redesign)
        L1(5, -35), // PLACEHOLDER(drawing)
        L2(17.5, -35), // PLACEHOLDER(funnel-redesign)
        L3(33.3, -35),
        L4(65.25, -52),
        ALGAE_LOW(8.8, 46),
        ALGAE_HIGH(24.6, 46),
        ALGAE_CARRY(1.75, 45),
        NET(65.25, 75),
        PROCESSOR(1.75, 15); // PLACEHOLDER(drawing)

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
