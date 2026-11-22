package frc.robot.subsystems.coral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.config.Ports.CanId;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.CoralK;

/** One Kraken X60 on the coral rollers and a CANrange looking across the coral's seat. */
public class CoralRollersIOTalonFX implements CoralRollersIO, AutoCloseable {
    private static final double SIGNAL_HZ = 100;

    protected final TalonFX motor;
    protected final CANrange range;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorAmps;
    private final StatusSignal<Distance> distance;

    // Open-loop roller voltage never needs FOC's extra torque, but it is free with the licence.
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.PRO_LICENSED);

    public CoralRollersIOTalonFX(CanId motorId, CanId rangeId) {
        motor = new TalonFX(motorId.id(), new CANBus(motorId.bus()));
        range = new CANrange(rangeId.id(), new CANBus(rangeId.bus()));

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = CoralK.GEAR_RATIO;
        config.CurrentLimits.StatorCurrentLimit = CoralK.STATOR_LIMIT_A;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);

        var rangeConfig = new CANrangeConfiguration();
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        range.getConfigurator().apply(rangeConfig);

        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        statorAmps = motor.getStatorCurrent();
        distance = range.getDistance();
        BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_HZ, velocity, appliedVolts, statorAmps, distance);
    }

    @Override
    public void updateInputs(CoralRollersIOInputs in) {
        in.connected = BaseStatusSignal.refreshAll(velocity, appliedVolts, statorAmps).isOK();
        in.rangeConnected = BaseStatusSignal.refreshAll(distance).isOK();
        in.velocityRps = velocity.getValueAsDouble();
        in.appliedVolts = appliedVolts.getValueAsDouble();
        in.statorAmps = statorAmps.getValueAsDouble();
        in.rangeM = distance.getValueAsDouble();
    }

    @Override
    public void setVolts(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void close() {
        motor.close();
        range.close();
    }
}
