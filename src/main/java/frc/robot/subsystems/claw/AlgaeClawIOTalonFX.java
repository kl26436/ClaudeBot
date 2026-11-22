package frc.robot.subsystems.claw;

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
import frc.robot.config.RobotConstants.ClawK;

/**
 * Two Kraken X60s on the algae claw, one per roller, and a CANrange. The rollers face each other, so
 * the two motors are inverted against each other in config: the same voltage runs both inward.
 */
public class AlgaeClawIOTalonFX implements AlgaeClawIO, AutoCloseable {
    private static final double SIGNAL_HZ = 100;

    protected final TalonFX front;
    protected final TalonFX rear;
    protected final CANrange range;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> frontStator;
    private final StatusSignal<Current> rearStator;
    private final StatusSignal<Distance> distance;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.PRO_LICENSED);

    public AlgaeClawIOTalonFX(CanId frontId, CanId rearId, CanId rangeId) {
        front = new TalonFX(frontId.id(), new CANBus(frontId.bus()));
        rear = new TalonFX(rearId.id(), new CANBus(rearId.bus()));
        range = new CANrange(rangeId.id(), new CANBus(rangeId.bus()));

        front.getConfigurator().apply(motorConfig(InvertedValue.CounterClockwise_Positive));
        rear.getConfigurator().apply(motorConfig(InvertedValue.Clockwise_Positive));

        var rangeConfig = new CANrangeConfiguration();
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        range.getConfigurator().apply(rangeConfig);

        velocity = front.getVelocity();
        appliedVolts = front.getMotorVoltage();
        frontStator = front.getStatorCurrent();
        rearStator = rear.getStatorCurrent();
        distance = range.getDistance();
        BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_HZ, velocity, appliedVolts, frontStator, rearStator, distance);
    }

    private static TalonFXConfiguration motorConfig(InvertedValue inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted;
        config.Feedback.SensorToMechanismRatio = ClawK.GEAR_RATIO;
        config.CurrentLimits.StatorCurrentLimit = ClawK.STATOR_LIMIT_A;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;
    }

    @Override
    public void updateInputs(AlgaeClawIOInputs in) {
        boolean frontOk = BaseStatusSignal.refreshAll(velocity, appliedVolts, frontStator).isOK();
        boolean rearOk = BaseStatusSignal.refreshAll(rearStator).isOK();
        in.connected = frontOk && rearOk;
        in.rangeConnected = BaseStatusSignal.refreshAll(distance).isOK();
        in.velocityRps = velocity.getValueAsDouble();
        in.appliedVolts = appliedVolts.getValueAsDouble();
        in.statorAmps = Math.max(Math.abs(frontStator.getValueAsDouble()), Math.abs(rearStator.getValueAsDouble()));
        in.rangeM = distance.getValueAsDouble();
    }

    @Override
    public void setVolts(double volts) {
        front.setControl(voltageRequest.withOutput(volts));
        rear.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void close() {
        front.close();
        rear.close();
        range.close();
    }
}
