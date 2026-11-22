package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.Ports;
import frc.robot.config.RobotConstants.WristK;

/** Wrist on simulated Phoenix devices, with a WPILib {@link SingleJointedArmSim} supplying the physics. */
public class WristIOSim extends WristIOTalonFX {
    private static final double LBIN_TO_NM = 0.1129848;
    /** The gravity constant {@link SingleJointedArmSim} uses internally. */
    private static final double MODEL_G = 9.8;
    /** PWM frame length of a REV Through Bore encoder (975.6 Hz). */
    private static final double PWM_PERIOD_S = 1.0 / 975.6;

    private final TalonFXSimState motorSim;
    private final CANdiSimState candiSim;
    private SingleJointedArmSim physics;
    private boolean encoderConnected = true;

    public WristIOSim() {
        super(Ports.WRIST, Ports.WRIST_CANDI);
        motorSim = motor.getSimState();
        candiSim = candi.getSimState();
        physics = build(WristK.GRAVITY_LBIN_EMPTY, Math.toRadians(WristK.MIN_DEG), 0);
        write();
    }

    /**
     * The WPILib model applies gravity as 3g/(2L) cos(angle), so the arm length is chosen to make the
     * torque at level equal to the one asked for: L = 3 g J / (2 torque).
     */
    private static SingleJointedArmSim build(double gravityLbIn, double angleRad, double velocityRadPerS) {
        double lengthM = 3 * MODEL_G * WristK.MOI_KGM2 / (2 * gravityLbIn * LBIN_TO_NM);
        var sim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                WristK.GEAR_RATIO,
                WristK.MOI_KGM2,
                lengthM,
                Math.toRadians(WristK.HARD_MIN_DEG),
                Math.toRadians(WristK.HARD_MAX_DEG),
                true,
                angleRad);
        sim.setState(angleRad, velocityRadPerS);
        return sim;
    }

    /** Advance the physics by {@code dtSeconds} using the voltage the motor is commanding. */
    public void simulate(double dtSeconds) {
        double battery = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(battery);
        candiSim.setSupplyVoltage(battery);
        physics.setInputVoltage(motorSim.getMotorVoltage());
        physics.update(dtSeconds);
        write();
    }

    /** Teleport the wrist, at rest, to an angle in degrees. */
    public void setWristDeg(double deg) {
        physics.setState(Math.toRadians(deg), 0);
        write();
    }

    /** Change the gravity torque at level, for example to add a game piece. */
    public void setGravityTorqueLbIn(double lbIn) {
        physics = build(lbIn, physics.getAngleRads(), physics.getVelocityRadPerSec());
    }

    /** Unplug or replug the absolute encoder's PWM signal. */
    public void setEncoderConnected(boolean connected) {
        encoderConnected = connected;
        write();
    }

    /** Where the wrist really is, whatever the sensors say. */
    public double trueWristDeg() {
        return Math.toDegrees(physics.getAngleRads());
    }

    private void write() {
        double rotations = Units.radiansToRotations(physics.getAngleRads());
        double rps = Units.radiansToRotations(physics.getVelocityRadPerSec());
        motorSim.setRawRotorPosition(rotations * WristK.GEAR_RATIO);
        motorSim.setRotorVelocity(rps * WristK.GEAR_RATIO);
        candiSim.setPwm1Connected(encoderConnected);
        candiSim.setPwm1RiseRise(encoderConnected ? PWM_PERIOD_S : 0);
        candiSim.setPwm1Position(rotations);
        candiSim.setPwm1Velocity(rps);
    }
}
