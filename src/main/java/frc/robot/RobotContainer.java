package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.ScoreCommands;
import frc.robot.config.Ports;
import frc.robot.config.RobotConstants;
import frc.robot.field.Reef;
import frc.robot.field.Station;
import frc.robot.sim.SimulatedRobot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.subsystems.claw.AlgaeClaw;
import frc.robot.subsystems.claw.AlgaeClawIO;
import frc.robot.subsystems.claw.AlgaeClawIOSim;
import frc.robot.subsystems.claw.AlgaeClawIOTalonFX;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.coral.CoralRollersIO;
import frc.robot.subsystems.coral.CoralRollersIOSim;
import frc.robot.subsystems.coral.CoralRollersIOTalonFX;
import frc.robot.subsystems.drive.SwerveIO;
import frc.robot.subsystems.drive.SwerveIOCTRE;
import frc.robot.subsystems.drive.SwerveIOSim;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.CameraModel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import java.util.ArrayList;
import java.util.List;

/**
 * Builds the subsystems for the run mode (real devices, simulated devices, or empty IO for log
 * replay) and binds the driver's controller. Every binding is in {@link #bindControls}.
 */
public class RobotContainer {
    /** NetworkTables name of the Limelight. */
    static final String LIMELIGHT_NAME = "limelight";

    /** Speed cap for driver-triggered scoring and pickup drives. */
    static final double TELEOP_ALIGN_SPEED_MPS = 3.0; // PLACEHOLDER(tuning)

    static final double STICK_DEADBAND = 0.1;

    private final SwerveSubsystem drive;
    private final Elevator elevator;
    private final Wrist wrist;
    private final CoralRollers coral;
    private final AlgaeClaw claw;
    private final Superstructure superstructure;
    private final Vision vision;
    private final SimulatedRobot sim;
    private final AutoChooser autoChooser;
    private final List<AutoCloseable> closeables = new ArrayList<>();
    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        SwerveIO driveIo;
        ElevatorIO elevatorIo;
        WristIO wristIo;
        CoralRollersIO coralIo;
        AlgaeClawIO clawIo;
        VisionIO visionIo;
        SwerveIOSim driveSim = null;
        ElevatorIOSim elevatorSim = null;
        WristIOSim wristSim = null;
        CoralRollersIOSim coralSim = null;
        AlgaeClawIOSim clawSim = null;

        switch (RobotConstants.currentMode()) {
            case REAL -> {
                driveIo = keep(new SwerveIOCTRE());
                elevatorIo = keep(new ElevatorIOTalonFX(Ports.ELEVATOR_LEFT, Ports.ELEVATOR_RIGHT, Ports.ELEVATOR_HOME_DIO));
                wristIo = keep(new WristIOTalonFX(Ports.WRIST, Ports.WRIST_CANDI));
                coralIo = keep(new CoralRollersIOTalonFX(Ports.CORAL_ROLLERS, Ports.CORAL_RANGE));
                clawIo = keep(new AlgaeClawIOTalonFX(Ports.CLAW_FRONT, Ports.CLAW_REAR, Ports.ALGAE_RANGE));
                visionIo = new VisionIOLimelight(LIMELIGHT_NAME);
            }
            case SIM -> {
                driveSim = keep(new SwerveIOSim());
                elevatorSim = keep(new ElevatorIOSim());
                wristSim = keep(new WristIOSim());
                coralSim = keep(new CoralRollersIOSim());
                clawSim = keep(new AlgaeClawIOSim());
                driveIo = driveSim;
                elevatorIo = elevatorSim;
                wristIo = wristSim;
                coralIo = coralSim;
                clawIo = clawSim;
                visionIo = new VisionIOSim(driveSim::truePose, CameraModel.limelight4());
            }
            default -> {
                // Replay: inputs come from the log.
                driveIo = new SwerveIO() {};
                elevatorIo = new ElevatorIO() {};
                wristIo = new WristIO() {};
                coralIo = new CoralRollersIO() {};
                clawIo = new AlgaeClawIO() {};
                visionIo = new VisionIO() {};
            }
        }

        drive = new SwerveSubsystem(driveIo);
        elevator = new Elevator(elevatorIo);
        wrist = new Wrist(wristIo);
        coral = new CoralRollers(coralIo);
        claw = new AlgaeClaw(clawIo);
        superstructure = new Superstructure(elevator, wrist, coral, claw);
        vision = new Vision(visionIo, drive);
        sim = driveSim == null
                ? null
                : new SimulatedRobot(driveSim, elevatorSim, wristSim, coralSim, clawSim, superstructure, coral, claw);

        drive.setSpeedScaleSource(superstructure::driveSpeedScale);
        autoChooser = new AutoChooser(this);
        bindControls();
    }

    private <T extends AutoCloseable> T keep(T device) {
        closeables.add(device);
        return device;
    }

    private Alliance allianceOrBlue() {
        return RobotState.get().alliance().orElse(Alliance.Blue);
    }

    /** The driver controls table of the design, one line per row. */
    private void bindControls() {
        // Left stick drives field-relative from the driver's side of the field; right stick turns.
        drive.setDefaultCommand(drive.run(() -> {
            double sign = allianceOrBlue() == Alliance.Red ? -1 : 1;
            double x = shape(-controller.getLeftY()) * sign;
            double y = shape(-controller.getLeftX()) * sign;
            double omega = shape(-controller.getRightX());
            drive.setTeleopInput(x, y, omega);
        }));

        controller.y().onTrue(Commands.runOnce(() -> RobotState.get().selectLevel(Wanted.L4)));
        controller.x().onTrue(Commands.runOnce(() -> RobotState.get().selectLevel(Wanted.L3)));
        controller.b().onTrue(Commands.runOnce(() -> RobotState.get().selectLevel(Wanted.L2)));
        controller.a().onTrue(Commands.runOnce(() -> RobotState.get().selectLevel(Wanted.L1)));

        controller.rightTrigger().whileTrue(scoreOnNearestFace(false));
        controller.leftTrigger().whileTrue(scoreOnNearestFace(true));
        controller.rightBumper().whileTrue(ScoreCommands.stationPickup(
                this,
                () -> Station.nearest(allianceOrBlue(), drive.pose().getTranslation()),
                TELEOP_ALIGN_SPEED_MPS,
                true));
        controller.leftBumper().whileTrue(ScoreCommands.algaeFromReef(
                this, () -> Reef.nearestFace(allianceOrBlue(), drive.pose().getTranslation())));
        controller.povUp().onTrue(ScoreCommands.netScore(this));
        controller.povDown().onTrue(ScoreCommands.processorScore(this));
        controller.back().onTrue(Commands.runOnce(() -> superstructure.setWanted(Wanted.STOW), superstructure, drive));
        controller.start().onTrue(Commands.runOnce(this::resetHeading));
    }

    private Command scoreOnNearestFace(boolean left) {
        return ScoreCommands.alignAndScore(
                this,
                () -> {
                    int face = Reef.nearestFace(allianceOrBlue(), drive.pose().getTranslation());
                    return Reef.Branch.values()[2 * face + (left ? 0 : 1)];
                },
                () -> RobotState.get().selectedLevel(),
                TELEOP_ALIGN_SPEED_MPS,
                true);
    }

    /** Declare the robot to be facing away from its own driver station. */
    private void resetHeading() {
        var forward = allianceOrBlue() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero;
        drive.resetPose(new Pose2d(drive.pose().getTranslation(), forward));
    }

    private static double shape(double stick) {
        double v = MathUtil.applyDeadband(stick, STICK_DEADBAND);
        return Math.copySign(v * v, v);
    }

    public SwerveSubsystem drive() {
        return drive;
    }

    public Superstructure superstructure() {
        return superstructure;
    }

    public Vision vision() {
        return vision;
    }

    public Elevator elevator() {
        return elevator;
    }

    public Wrist wrist() {
        return wrist;
    }

    public CoralRollers coral() {
        return coral;
    }

    public AlgaeClaw claw() {
        return claw;
    }

    public AutoChooser autoChooser() {
        return autoChooser;
    }

    /** The simulated world, or null outside simulation. */
    public SimulatedRobot sim() {
        return sim;
    }

    /** Release every device and alert, for tests that build more than one robot. */
    public void close() {
        superstructure.close();
        elevator.close();
        wrist.close();
        coral.close();
        claw.close();
        for (var c : closeables) {
            try {
                c.close();
            } catch (Exception e) {
                throw new IllegalStateException("could not close " + c, e);
            }
        }
    }
}
