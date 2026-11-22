package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.RobotConstants.WristK;
import frc.robot.field.Station;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.claw.AlgaeClaw;
import frc.robot.subsystems.claw.AlgaeClawIOSim;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.coral.CoralRollersIOSim;
import frc.robot.subsystems.drive.SwerveIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.wrist.WristIOSim;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's world in simulation: steps every physics model, plays the human player at the coral
 * stations, lets a coral leave when the rollers push it out, and publishes the mechanism poses
 * AdvantageScope draws.
 *
 * <p>{@link #simulate} may run on its own thread (simulateJava uses a 5 ms notifier); it and the
 * logging in {@link #periodic} are synchronized.
 */
public class SimulatedRobot extends SubsystemBase {
    /** Time from the robot settling at a station to the coral arriving in the funnel. */
    public static final double HUMAN_DELAY_S = 0.7;

    /** How close to a pickup pose counts as at the station. */
    static final double STATION_TOL_M = 0.15;

    static final double STATION_TOL_DEG = 5;

    /** Pushing for this long gets a coral clear of the rollers. */
    static final double CORAL_EJECT_S = 0.1;

    private final SwerveIOSim drive;
    private final ElevatorIOSim elevator;
    private final WristIOSim wrist;
    private final CoralRollersIOSim coralIo;
    private final AlgaeClawIOSim clawIo;
    private final Superstructure superstructure;
    private final CoralRollers coral;
    private final AlgaeClaw claw;

    private double clockS = 0;
    private double atStationSinceS = Double.NaN;
    private double ejectingSinceS = Double.NaN;

    public SimulatedRobot(
            SwerveIOSim drive,
            ElevatorIOSim elevator,
            WristIOSim wrist,
            CoralRollersIOSim coralIo,
            AlgaeClawIOSim clawIo,
            Superstructure superstructure,
            CoralRollers coral,
            AlgaeClaw claw) {
        this.drive = drive;
        this.elevator = elevator;
        this.wrist = wrist;
        this.coralIo = coralIo;
        this.clawIo = clawIo;
        this.superstructure = superstructure;
        this.coral = coral;
        this.claw = claw;
    }

    /** Advance the whole robot and the field by {@code dt} seconds. */
    public synchronized void simulate(double dt) {
        clockS += dt;
        drive.simulate(dt);
        elevator.simulate(dt);
        wrist.simulate(dt);
        coralIo.simulate(dt);
        clawIo.simulate(dt);
        humanPlayer();
        coralRelease();
    }

    /** Simulated seconds since the robot was built. */
    public synchronized double clockS() {
        return clockS;
    }

    private void humanPlayer() {
        if (coralIo.coralPresent() || superstructure.wanted() != Superstructure.Wanted.STATION_INTAKE || !atAStation()) {
            atStationSinceS = Double.NaN;
            return;
        }
        if (Double.isNaN(atStationSinceS)) {
            atStationSinceS = clockS;
        }
        if (clockS - atStationSinceS >= HUMAN_DELAY_S) {
            setCoral(true);
            atStationSinceS = Double.NaN;
        }
    }

    private boolean atAStation() {
        Pose2d robot = drive.truePose();
        for (Alliance a : Alliance.values()) {
            for (Station.Side s : Station.Side.values()) {
                Pose2d p = Station.pickupPose(a, s);
                if (robot.getTranslation().getDistance(p.getTranslation()) <= STATION_TOL_M
                        && Math.abs(robot.getRotation().minus(p.getRotation()).getDegrees()) <= STATION_TOL_DEG) {
                    return true;
                }
            }
        }
        return false;
    }

    /** A coral pushed out for long enough is gone. */
    private void coralRelease() {
        if (!coralIo.coralPresent() || !coral.isEjecting()) {
            ejectingSinceS = Double.NaN;
            return;
        }
        if (Double.isNaN(ejectingSinceS)) {
            ejectingSinceS = clockS;
        }
        if (clockS - ejectingSinceS >= CORAL_EJECT_S) {
            onCoralLeaves();
            setCoral(false);
            ejectingSinceS = Double.NaN;
        }
    }

    /** Called on the tick a coral leaves the rollers. */
    protected void onCoralLeaves() {}

    private void setCoral(boolean present) {
        coralIo.setCoralPresent(present);
        wrist.setGravityTorqueLbIn(present ? WristK.GRAVITY_LBIN_CORAL : WristK.GRAVITY_LBIN_EMPTY);
    }

    /** Put a coral in the rollers, as loaded before a match. */
    public synchronized void preloadCoral() {
        setCoral(true);
    }

    public synchronized Pose2d truePose() {
        return drive.truePose();
    }

    /** Teleport the robot, at rest. Odometry is not told. */
    public synchronized void setTruePose(Pose2d pose) {
        drive.setTruePose(pose);
    }

    /**
     * Stage A, stage B, carriage and wrist, in the robot frame, for AdvantageScope's model. The
     * stages rise a third and two thirds as far as the carriage; the wrist turns about its pivot.
     */
    public synchronized Pose3d[] componentPoses() {
        double h = Units.inchesToMeters(elevator.trueCarriageIn());
        double pitch = -Math.toRadians(wrist.trueWristDeg()); // WPILib pitch is positive nose-down
        return new Pose3d[] {
            new Pose3d(0, 0, h / 3, new Rotation3d()),
            new Pose3d(0, 0, 2 * h / 3, new Rotation3d()),
            new Pose3d(0, 0, h, new Rotation3d()),
            new Pose3d(
                    Units.inchesToMeters(ElevatorK.PIVOT_X_IN),
                    0,
                    Units.inchesToMeters(ElevatorK.PIVOT_REST_HEIGHT_IN) + h,
                    new Rotation3d(0, pitch, 0)),
        };
    }

    @Override
    public synchronized void periodic() {
        Logger.recordOutput("Mechanism/Components", componentPoses());
        Logger.recordOutput("Sim/TruePose", drive.truePose());
        Logger.recordOutput("Sim/HasCoral", coralIo.coralPresent());
        Logger.recordOutput("Sim/HasAlgae", clawIo.algaePresent());
    }
}
