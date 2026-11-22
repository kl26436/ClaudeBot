// Adapted from FRC Team 2910's 2026CompetitionRobot-Public (MIT licence; see NOTICE): SwerveIOSim and
// MapleSimSwerveDrivetrain, which inject maple-sim physics into CTRE's simulated devices.
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.DriveK;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

/**
 * The CTRE drivetrain on simulated devices, with maple-sim's 2025 field supplying the physics. Each
 * {@link #simulate} steps the arena once and maple-sim writes the module states straight into the
 * simulated TalonFXs, CANcoders and Pigeon 2.
 *
 * <p>Every instance builds its own {@link Arena2025Reefscape} and makes it the arena instance, so
 * one test's robot never shares a field with the next one's.
 */
public class SwerveIOSim extends SwerveIOCTRE {
    private static final double WHEEL_COF = 1.2; // PLACEHOLDER(field-check)

    /**
     * Odometry rate in simulation. CTRE's 250 Hz on twelve simulated devices loads Phoenix's
     * simulator enough that other devices' remote-sensor frames go stale; 100 Hz is plenty here.
     */
    private static final double SIM_ODOMETRY_HZ = 100;

    // Simulation-only module adjustments, as 2910 uses with maple-sim.
    private static final double SIM_STEER_KP = 70;
    private static final double SIM_STEER_KD = 4.5;
    private static final double SIM_DRIVE_FRICTION_V = 0.1;
    private static final double SIM_STEER_FRICTION_V = 0.15;
    private static final double SIM_STEER_INERTIA_KGM2 = 0.05;

    private final Arena2025Reefscape arena;
    private final SwerveDriveSimulation physics;
    private final Pigeon2SimState pigeonSim;

    public SwerveIOSim() {
        this(simulationModules());
    }

    private SwerveIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] modules) {
        super(TunerConstants.DRIVETRAIN, SIM_ODOMETRY_HZ, modules);
        arena = new Arena2025Reefscape();
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.overrideSimulationTimings(Seconds.of(0.005), 1);

        DCMotor motor = RobotConstants.PRO_LICENSED ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);
        var m = modules[0];
        var config = DriveTrainSimulationConfig.Default()
                .withRobotMass(Units.Pounds.of(DriveK.ROBOT_MASS_LB))
                .withBumperSize(
                        Units.Inches.of(2 * DriveK.BUMPER_FACE_FROM_CENTRE_IN),
                        Units.Inches.of(2 * DriveK.BUMPER_FACE_FROM_CENTRE_IN))
                .withGyro(COTS.ofPigeon2())
                .withCustomModuleTranslations(getModuleLocations())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        motor,
                        motor,
                        m.DriveMotorGearRatio,
                        m.SteerMotorGearRatio,
                        Volts.of(m.DriveFrictionVoltage),
                        Volts.of(m.SteerFrictionVoltage),
                        Meters.of(m.WheelRadius),
                        KilogramSquareMeters.of(m.SteerInertia),
                        WHEEL_COF));
        physics = new SwerveDriveSimulation(config, new Pose2d());
        var moduleSims = physics.getModules();
        var ctreModules = getModules();
        for (int i = 0; i < moduleSims.length; i++) {
            moduleSims[i].useDriveMotorController(new TalonFXMotor(ctreModules[i].getDriveMotor()));
            moduleSims[i].useSteerMotorController(
                    new TalonFXWithCANcoder(ctreModules[i].getSteerMotor(), ctreModules[i].getEncoder()));
        }
        arena.addDriveTrainSimulation(physics);
        pigeonSim = getPigeon2().getSimState();
    }

    /**
     * Module constants adjusted for maple-sim: no offsets, inversions or coupling (the simulated
     * modules have none to undo), and steer gains and friction that suit them.
     */
    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            simulationModules() {
        var modules = TunerConstants.modules();
        for (var m : modules) {
            Slot0Configs g = m.SteerMotorGains;
            m.withEncoderOffset(0)
                    // maple-sim's wheels have no steer-to-drive coupling, so CTRE must not correct for one.
                    .withCouplingGearRatio(0)
                    .withDriveMotorInverted(false)
                    .withSteerMotorInverted(false)
                    .withEncoderInverted(false)
                    // A copy: the robot's gains object is shared and must not change.
                    .withSteerMotorGains(new Slot0Configs()
                            .withKP(SIM_STEER_KP)
                            .withKD(SIM_STEER_KD)
                            .withKS(g.kS)
                            .withKV(g.kV)
                            .withKA(g.kA)
                            .withStaticFeedforwardSign(g.StaticFeedforwardSign))
                    .withDriveFrictionVoltage(SIM_DRIVE_FRICTION_V)
                    .withSteerFrictionVoltage(SIM_STEER_FRICTION_V)
                    .withSteerInertia(SIM_STEER_INERTIA_KGM2);
        }
        return modules;
    }

    /** Step the field by {@code dt} seconds and write the result into the simulated devices. */
    public void simulate(double dt) {
        SimulatedArena.overrideSimulationTimings(Seconds.of(dt), 1);
        arena.simulationPeriodic();
        pigeonSim.setRawYaw(physics.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
                physics.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
    }

    /** Where the robot really is on the field. */
    public Pose2d truePose() {
        return physics.getSimulatedDriveTrainPose();
    }

    /** Teleport the robot, at rest. Odometry is not told; call {@code resetPose} for that. */
    public void setTruePose(Pose2d pose) {
        physics.setSimulationWorldPose(pose);
        physics.setRobotSpeeds(new ChassisSpeeds());
        pigeonSim.setRawYaw(pose.getRotation().getMeasure());
    }

    /** The field this robot is driving on. */
    public Arena2025Reefscape arena() {
        return arena;
    }

    /** The maple-sim body, for game piece interactions. */
    public SwerveDriveSimulation physics() {
        return physics;
    }

    @Override
    public void close() {
        super.close();
        arena.shutDown();
    }

    /** Feeds maple-sim's drive motor state into a simulated TalonFX and returns its output voltage. */
    private static class TalonFXMotor implements SimulatedMotorController {
        private final TalonFXSimState sim;

        TalonFXMotor(TalonFX motor) {
            sim = motor.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle, AngularVelocity encoderVelocity) {
            sim.setRawRotorPosition(encoderAngle);
            sim.setRotorVelocity(encoderVelocity);
            sim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return sim.getMotorVoltageMeasure();
        }
    }

    /** As {@link TalonFXMotor}, and also feeds the module angle into the steer CANcoder. */
    private static final class TalonFXWithCANcoder extends TalonFXMotor {
        private final CANcoderSimState encoderSim;

        TalonFXWithCANcoder(TalonFX motor, CANcoder encoder) {
            super(motor);
            encoderSim = encoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle, AngularVelocity encoderVelocity) {
            encoderSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            encoderSim.setRawPosition(mechanismAngle);
            encoderSim.setVelocity(mechanismVelocity);
            return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }
}
