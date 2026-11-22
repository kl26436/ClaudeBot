package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Superstructure;
import java.util.Optional;

/**
 * What the rest of the code needs to know about the robot as a whole: where it is, how fast it is
 * going, which alliance it is on, and which reef level the driver has picked. The drivetrain writes
 * the pose and speeds once per loop.
 */
public final class RobotState {
    private static final RobotState INSTANCE = new RobotState();

    private volatile Pose2d pose = new Pose2d();
    private volatile ChassisSpeeds fieldSpeeds = new ChassisSpeeds();
    private volatile Superstructure.Wanted selectedLevel = Superstructure.Wanted.L4;

    private RobotState() {}

    public static RobotState get() {
        return INSTANCE;
    }

    /** Called by the drivetrain each loop with its estimate and field-relative speeds. */
    public void update(Pose2d estimate, ChassisSpeeds fieldRelativeSpeeds) {
        pose = estimate;
        fieldSpeeds = fieldRelativeSpeeds;
    }

    /** Estimated pose on the field, blue origin. */
    public Pose2d pose() {
        return pose;
    }

    /** Field-relative chassis speeds, blue origin. */
    public ChassisSpeeds speeds() {
        return fieldSpeeds;
    }

    /** The alliance from the driver station, or empty while it is not known. */
    public Optional<Alliance> alliance() {
        return DriverStation.getAlliance();
    }

    /** The reef level the driver has picked for the next coral. */
    public Superstructure.Wanted selectedLevel() {
        return selectedLevel;
    }

    /** Pick the reef level for the next coral. Only L1 to L4 are accepted. */
    public void selectLevel(Superstructure.Wanted level) {
        switch (level) {
            case L1, L2, L3, L4 -> selectedLevel = level;
            default -> throw new IllegalArgumentException("not a reef level: " + level);
        }
    }
}
