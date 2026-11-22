package frc.robot.subsystems.coral;

import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.config.Ports;
import frc.robot.config.RobotConstants.CoralK;

/**
 * Coral rollers on simulated Phoenix devices. A seated coral stalls the roller and puts the CANrange
 * reading under the detection distance.
 *
 * <p>Phoenix simulates the CANrange's distance but cannot take it off the bus, so a disconnected
 * sensor is simulated here, at the IO layer.
 */
public class CoralRollersIOSim extends CoralRollersIOTalonFX {
    /** CANrange reading with a coral seated, and with the seat empty (looking at the far side plate). */
    static final double RANGE_WITH_CORAL_M = 0.02;

    static final double RANGE_EMPTY_M = 0.30;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final TalonFXSimState motorSim;
    private final CANrangeSimState rangeSim;
    private final DCMotorSim physics = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, CoralK.MOI_KGM2, CoralK.GEAR_RATIO), GEARBOX);
    private boolean coralPresent = false;
    private boolean rangeConnected = true;

    public CoralRollersIOSim() {
        super(Ports.CORAL_ROLLERS, Ports.CORAL_RANGE);
        motorSim = motor.getSimState();
        rangeSim = range.getSimState();
        write();
    }

    /** Advance the physics by {@code dt} seconds using the voltage the motor is commanding. */
    public void simulate(double dt) {
        double battery = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(battery);
        rangeSim.setSupplyVoltage(battery);
        physics.setInputVoltage(motorSim.getMotorVoltage());
        physics.update(dt);
        if (coralPresent) {
            // A seated coral is against the hard stop: the roller cannot turn.
            physics.setState(physics.getAngularPositionRad(), 0);
        }
        write();
    }

    public void setCoralPresent(boolean present) {
        coralPresent = present;
        write();
    }

    public boolean coralPresent() {
        return coralPresent;
    }

    /** Take the CANrange off the bus, or put it back. */
    public void setRangeConnected(boolean connected) {
        rangeConnected = connected;
    }

    @Override
    public void updateInputs(CoralRollersIOInputs in) {
        super.updateInputs(in);
        if (!rangeConnected) {
            in.rangeConnected = false;
        }
    }

    private void write() {
        motorSim.setRawRotorPosition(Units.radiansToRotations(physics.getAngularPositionRad()) * CoralK.GEAR_RATIO);
        motorSim.setRotorVelocity(Units.radiansToRotations(physics.getAngularVelocityRadPerSec()) * CoralK.GEAR_RATIO);
        rangeSim.setDistance(coralPresent ? RANGE_WITH_CORAL_M : RANGE_EMPTY_M);
    }
}
