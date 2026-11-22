package frc.robot.subsystems.claw;

import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.config.Ports;
import frc.robot.config.RobotConstants.ClawK;

/**
 * Algae claw on simulated Phoenix devices: one roller model per motor, each in its own motor's frame
 * so the inversion needs no special handling. A held algae stalls both rollers and puts the CANrange
 * reading under the detection distance.
 *
 * <p>Phoenix cannot take a simulated CANrange off the bus, so a disconnected sensor is simulated here.
 */
public class AlgaeClawIOSim extends AlgaeClawIOTalonFX {
    static final double RANGE_WITH_ALGAE_M = 0.03;
    static final double RANGE_EMPTY_M = 0.45;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final TalonFXSimState frontSim;
    private final TalonFXSimState rearSim;
    private final CANrangeSimState rangeSim;
    private final DCMotorSim frontPhysics = roller();
    private final DCMotorSim rearPhysics = roller();
    private boolean algaePresent = false;
    private boolean rangeConnected = true;

    public AlgaeClawIOSim() {
        super(Ports.CLAW_FRONT, Ports.CLAW_REAR, Ports.ALGAE_RANGE);
        frontSim = front.getSimState();
        rearSim = rear.getSimState();
        rangeSim = range.getSimState();
        write();
    }

    private static DCMotorSim roller() {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, ClawK.MOI_KGM2, ClawK.GEAR_RATIO), GEARBOX);
    }

    /** Advance the physics by {@code dt} seconds using the voltage each motor is commanding. */
    public void simulate(double dt) {
        double battery = RobotController.getBatteryVoltage();
        frontSim.setSupplyVoltage(battery);
        rearSim.setSupplyVoltage(battery);
        rangeSim.setSupplyVoltage(battery);
        step(frontPhysics, frontSim, dt);
        step(rearPhysics, rearSim, dt);
        write();
    }

    private void step(DCMotorSim physics, TalonFXSimState motorSim, double dt) {
        physics.setInputVoltage(motorSim.getMotorVoltage());
        physics.update(dt);
        if (algaePresent) {
            // A held algae is squeezed between the rollers: they cannot turn.
            physics.setState(physics.getAngularPositionRad(), 0);
        }
    }

    public void setAlgaePresent(boolean present) {
        algaePresent = present;
        write();
    }

    public boolean algaePresent() {
        return algaePresent;
    }

    /** Take the CANrange off the bus, or put it back. */
    public void setRangeConnected(boolean connected) {
        rangeConnected = connected;
    }

    @Override
    public void updateInputs(AlgaeClawIOInputs in) {
        super.updateInputs(in);
        if (!rangeConnected) {
            in.rangeConnected = false;
        }
    }

    private void write() {
        writeRotor(frontPhysics, frontSim);
        writeRotor(rearPhysics, rearSim);
        rangeSim.setDistance(algaePresent ? RANGE_WITH_ALGAE_M : RANGE_EMPTY_M);
    }

    private static void writeRotor(DCMotorSim physics, TalonFXSimState motorSim) {
        motorSim.setRawRotorPosition(Units.radiansToRotations(physics.getAngularPositionRad()) * ClawK.GEAR_RATIO);
        motorSim.setRotorVelocity(Units.radiansToRotations(physics.getAngularVelocityRadPerSec()) * ClawK.GEAR_RATIO);
    }
}
