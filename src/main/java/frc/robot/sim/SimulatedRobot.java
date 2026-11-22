package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.RobotConstants.WristK;
import frc.robot.field.Reef;
import frc.robot.field.Station;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.subsystems.claw.AlgaeClaw;
import frc.robot.subsystems.claw.AlgaeClawIOSim;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.coral.CoralRollersIOSim;
import frc.robot.subsystems.drive.SwerveIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.wrist.WristIOSim;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's world in simulation: steps every physics model, plays the human player at the coral
 * stations, judges each game piece as it leaves the robot, and publishes the mechanism and game
 * piece poses AdvantageScope draws.
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

    /** Pushing for this long gets a coral clear of the rollers, or an algae clear of the claw. */
    static final double CORAL_EJECT_S = 0.1;

    static final double ALGAE_EJECT_S = 0.1;

    /** How close to a face's algae pose the claw must be to take its algae. */
    static final double ALGAE_TOL_IN = 2.0;

    /** Reef algae sit this far inside the face, at these heights (low and high). */
    static final double ALGAE_INSET_IN = 6.78;

    static final double ALGAE_LOW_IN = 35.0;
    static final double ALGAE_HIGH_IN = 50.9;

    /** Speed a released coral leaves the wrist at, for the picture only. */
    static final double CORAL_LAUNCH_MPS = 1.0;

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
    private double coralEjectingSinceS = Double.NaN;
    private double algaeEjectingSinceS = Double.NaN;

    /** When the present mode (auto, teleop, disabled) began, so scores carry match-phase times. */
    private double modeStartS = 0;

    private String lastMode = "";
    private final List<ScoreJudge.CoralScore> coralScored = new ArrayList<>();
    private final Set<String> scoredSpots = new HashSet<>();
    /** Algae still on each reef face, blue then red. */
    private final boolean[][] reefAlgae = {
        {true, true, true, true, true, true}, {true, true, true, true, true, true}
    };

    private int algaeInNet = 0;

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
        trackMode();
        humanPlayer();
        coralRelease();
        algaePickup();
        algaeRelease();
    }

    private void trackMode() {
        String mode = DriverStation.isDisabled() ? "disabled" : DriverStation.isAutonomous() ? "auto" : "teleop";
        if (!mode.equals(lastMode)) {
            lastMode = mode;
            modeStartS = clockS;
        }
    }

    /** Simulated seconds since the present mode began. */
    public synchronized double modeTimeS() {
        return clockS - modeStartS;
    }

    private void humanPlayer() {
        if (coralIo.coralPresent() || superstructure.wanted() != Wanted.STATION_INTAKE || !atAStation()) {
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

    /** A coral pushed out for long enough is gone: judge it on that tick. */
    private void coralRelease() {
        if (!coralIo.coralPresent() || !coral.isEjecting()) {
            coralEjectingSinceS = Double.NaN;
            return;
        }
        if (Double.isNaN(coralEjectingSinceS)) {
            coralEjectingSinceS = clockS;
        }
        if (clockS - coralEjectingSinceS >= CORAL_EJECT_S) {
            onCoralLeaves();
            setCoral(false);
            coralEjectingSinceS = Double.NaN;
        }
    }

    /** Judge the coral, record a score (each branch and level once), and launch it for the picture. */
    private void onCoralLeaves() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d pose = drive.truePose();
        double h = elevator.trueCarriageIn();
        double w = wrist.trueWristDeg();
        if (alliance.isPresent()) {
            ScoreJudge.judgeCoral(alliance.get(), pose, h, w, modeTimeS()).ifPresent(score -> {
                if (scoredSpots.add(score.alliance() + "/" + score.branch() + "/" + score.level())) {
                    coralScored.add(score);
                }
            });
        }
        drive.arena().addGamePieceProjectile(new ReefscapeCoralOnFly(
                pose.getTranslation(),
                new Translation2d(Units.inchesToMeters(ElevatorK.PIVOT_X_IN), 0),
                drive.physics().getDriveTrainSimulatedChassisSpeedsRobotRelative(),
                pose.getRotation(),
                edu.wpi.first.units.Units.Inches.of(ElevatorK.PIVOT_REST_HEIGHT_IN + h),
                edu.wpi.first.units.Units.MetersPerSecond.of(CORAL_LAUNCH_MPS),
                edu.wpi.first.units.Units.Degrees.of(w)));
    }

    /** The claw takes a face's algae when it is intaking at that face's height, on its algae pose. */
    private void algaePickup() {
        Wanted want = superstructure.wanted();
        if (clawIo.algaePresent() || (want != Wanted.ALGAE_LOW && want != Wanted.ALGAE_HIGH) || !superstructure.atPose()) {
            return;
        }
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }
        Pose2d robot = drive.truePose();
        int side = side(alliance.get());
        for (int face = 0; face < 6; face++) {
            if (Reef.algaeIsHigh(face) != (want == Wanted.ALGAE_HIGH) || !reefAlgae[side][face]) {
                continue;
            }
            double offM = robot.getTranslation().getDistance(Reef.algaePose(alliance.get(), face).getTranslation());
            if (Units.metersToInches(offM) <= ALGAE_TOL_IN) {
                reefAlgae[side][face] = false;
                setAlgae(true);
                return;
            }
        }
    }

    /** An algae pushed out for long enough is gone, into the net if the judge says so. */
    private void algaeRelease() {
        if (!clawIo.algaePresent() || !claw.isEjecting()) {
            algaeEjectingSinceS = Double.NaN;
            return;
        }
        if (Double.isNaN(algaeEjectingSinceS)) {
            algaeEjectingSinceS = clockS;
        }
        if (clockS - algaeEjectingSinceS >= ALGAE_EJECT_S) {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()
                    && ScoreJudge.judgeNet(alliance.get(), drive.truePose(), elevator.trueCarriageIn(), wrist.trueWristDeg())) {
                algaeInNet++;
            }
            setAlgae(false);
            algaeEjectingSinceS = Double.NaN;
        }
    }

    private static int side(Alliance a) {
        return a == Alliance.Blue ? 0 : 1;
    }

    private void setCoral(boolean present) {
        coralIo.setCoralPresent(present);
        updateWristLoad();
    }

    private void setAlgae(boolean present) {
        clawIo.setAlgaePresent(present);
        updateWristLoad();
    }

    /** The wrist carries whatever it holds; an algae is the heavier of the two. */
    private void updateWristLoad() {
        double load = clawIo.algaePresent()
                ? WristK.GRAVITY_LBIN_ALGAE
                : coralIo.coralPresent() ? WristK.GRAVITY_LBIN_CORAL : WristK.GRAVITY_LBIN_EMPTY;
        wrist.setGravityTorqueLbIn(load);
    }

    /**
     * Put the field back as it was at the start of a match: every reef algae in place, no scores, no
     * pieces on the robot or in the air. The robot itself stays where it is.
     */
    public synchronized void resetField() {
        coralScored.clear();
        scoredSpots.clear();
        for (boolean[] faces : reefAlgae) {
            java.util.Arrays.fill(faces, true);
        }
        algaeInNet = 0;
        atStationSinceS = Double.NaN;
        coralEjectingSinceS = Double.NaN;
        algaeEjectingSinceS = Double.NaN;
        coralIo.setCoralPresent(false);
        clawIo.setAlgaePresent(false);
        updateWristLoad();
        drive.arena().clearGamePieces();
    }

    /** Put a coral in the rollers, as loaded before a match. */
    public synchronized void preloadCoral() {
        setCoral(true);
    }

    /** Put an algae in the claw, for tests. */
    public synchronized void giveAlgae() {
        setAlgae(true);
    }

    /** Every coral scored, in order, each branch and level once. */
    public synchronized List<ScoreJudge.CoralScore> coralScored() {
        return List.copyOf(coralScored);
    }

    public synchronized int algaeInNet() {
        return algaeInNet;
    }

    /** Whether the face's algae is still on the reef. */
    public synchronized boolean reefAlgaePresent(Alliance a, int face) {
        return reefAlgae[side(a)][face];
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

    /** The wrist pivot on the field, where a held piece is drawn. */
    private Pose3d heldPiecePose() {
        return new Pose3d(drive.truePose()).transformBy(new Transform3d(new Pose3d(), componentPoses()[3]));
    }

    private static Pose3d onField(Pose2d at, double upIn) {
        return new Pose3d(at.getX(), at.getY(), Units.inchesToMeters(upIn), new Rotation3d());
    }

    /** Coral on the robot and on the branches it was scored on. */
    private Pose3d[] coralPoses() {
        List<Pose3d> out = new ArrayList<>();
        if (coralIo.coralPresent()) {
            out.add(heldPiecePose());
        }
        for (var s : coralScored) {
            for (var p : ScoreJudge.BRANCH_POINTS) {
                if (p.level() == s.level()) {
                    var at = Reef.scoringPose(s.alliance(), s.branch())
                            .transformBy(new Transform2d(Units.inchesToMeters(p.forwardIn()), 0, Rotation2d.kZero));
                    out.add(onField(at, p.upIn()));
                }
            }
        }
        return out.toArray(new Pose3d[0]);
    }

    /** Algae on the robot and still on the reef. */
    private Pose3d[] algaePoses() {
        List<Pose3d> out = new ArrayList<>();
        if (clawIo.algaePresent()) {
            out.add(heldPiecePose());
        }
        for (Alliance a : Alliance.values()) {
            for (int face = 0; face < 6; face++) {
                if (reefAlgae[side(a)][face]) {
                    var at = Reef.tagPose(a, face).transformBy(
                            new Transform2d(-Units.inchesToMeters(ALGAE_INSET_IN), 0, Rotation2d.kZero));
                    out.add(onField(at, Reef.algaeIsHigh(face) ? ALGAE_HIGH_IN : ALGAE_LOW_IN));
                }
            }
        }
        return out.toArray(new Pose3d[0]);
    }

    @Override
    public synchronized void periodic() {
        Logger.recordOutput("Mechanism/Components", componentPoses());
        Logger.recordOutput("Sim/TruePose", drive.truePose());
        Logger.recordOutput("Sim/Coral", coralPoses());
        Logger.recordOutput("Sim/Algae", algaePoses());
        Logger.recordOutput("Sim/Score/Coral", coralScored.size());
        Logger.recordOutput("Sim/Score/Net", algaeInNet);
    }
}
