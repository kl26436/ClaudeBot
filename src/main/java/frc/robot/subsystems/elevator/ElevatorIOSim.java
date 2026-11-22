package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.config.Ports;
import frc.robot.config.RobotConstants.ElevatorK;

/** Elevator on simulated Phoenix devices, with a WPILib {@link ElevatorSim} supplying the physics. */
public class ElevatorIOSim extends ElevatorIOTalonFX {
    private static final double LB_TO_KG = 0.45359237;
    private static final double HOME_SWITCH_IN = 0.05;

    private final ElevatorSim physics;
    private final TalonFXSimState leftSim;
    private final TalonFXSimState rightSim;
    private boolean homeSwitchBroken = false;

    public ElevatorIOSim() {
        super(Ports.ELEVATOR_LEFT, Ports.ELEVATOR_RIGHT, Ports.ELEVATOR_HOME_DIO);
        double carriageKg =
                (ElevatorK.STAGE_A_LB + 2 * ElevatorK.STAGE_B_LB + 3 * ElevatorK.CARRIAGE_GROUP_LB) / 3 * LB_TO_KG;
        // The drum circumference is the carriage travel per output turn, stages included.
        double drumRadiusM = Units.inchesToMeters(ElevatorK.SPROCKET_TEETH * ElevatorK.CHAIN_PITCH_IN * ElevatorK.STAGES)
                / (2 * Math.PI);
        physics = new ElevatorSim(
                DCMotor.getKrakenX60Foc(2),
                ElevatorK.GEAR_RATIO,
                carriageKg,
                drumRadiusM,
                0,
                Units.inchesToMeters(ElevatorK.TRAVEL_IN),
                true,
                0);
        leftSim = left.getSimState();
        rightSim = right.getSimState();
        writeRotor();
    }

    /** Advance the physics by {@code dtSeconds} using the voltage the left motor is commanding. */
    public void simulate(double dtSeconds) {
        double battery = RobotController.getBatteryVoltage();
        leftSim.setSupplyVoltage(battery);
        rightSim.setSupplyVoltage(battery);
        physics.setInput(leftSim.getMotorVoltage());
        physics.update(dtSeconds);
        writeRotor();
    }

    /** Teleport the carriage, at rest, to a height in inches. */
    public void setCarriageIn(double in) {
        physics.setState(Units.inchesToMeters(in), 0);
        writeRotor();
    }

    public void setHomeSwitchBroken(boolean broken) {
        homeSwitchBroken = broken;
    }

    private double heightIn() {
        return Units.metersToInches(physics.getPositionMeters());
    }

    private void writeRotor() {
        double rotations = heightIn() / ElevatorK.IN_PER_MOTOR_ROT;
        double rps = Units.metersToInches(physics.getVelocityMetersPerSecond()) / ElevatorK.IN_PER_MOTOR_ROT;
        leftSim.setRawRotorPosition(rotations);
        leftSim.setRotorVelocity(rps);
        rightSim.setRawRotorPosition(-rotations);
        rightSim.setRotorVelocity(-rps);
    }

    @Override
    protected boolean homeSwitchClosed() {
        return !homeSwitchBroken && heightIn() <= HOME_SWITCH_IN;
    }
}
