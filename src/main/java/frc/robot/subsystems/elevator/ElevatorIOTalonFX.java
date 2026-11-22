package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.Ports.CanId;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.ElevatorK;

/**
 * Two Kraken X60s on the elevator. The left motor is the leader and reports in inches of carriage
 * travel (the sensor-to-mechanism ratio folds the gearbox and chain stages into one factor); the
 * right motor follows it.
 */
public class ElevatorIOTalonFX implements ElevatorIO, AutoCloseable {
    // Control gains in volts. Units follow the mechanism: inches, inches per second.
    protected static final double KG = 0.46; // PLACEHOLDER(tuning)
    protected static final double KS = 0.0; // PLACEHOLDER(tuning)
    protected static final double KV = 0.0903; // PLACEHOLDER(tuning)
    protected static final double KA = 0.0012; // PLACEHOLDER(tuning)
    protected static final double KP = 0.5; // PLACEHOLDER(tuning)
    protected static final double KD = 0.01; // PLACEHOLDER(tuning)

    private static final double FAST_SIGNAL_HZ = 250;
    private static final double SLOW_SIGNAL_HZ = 50;

    protected final TalonFX left;
    protected final TalonFX right;
    private final DigitalInput homeInput;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorAmps;
    private final StatusSignal<Current> supplyAmps;
    private final StatusSignal<Angle> followerPosition;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.PRO_LICENSED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.PRO_LICENSED);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private boolean softLimitsEnabled = false;

    public ElevatorIOTalonFX(CanId leftId, CanId rightId, int homeDio) {
        left = new TalonFX(leftId.id(), new CANBus(leftId.bus()));
        right = new TalonFX(rightId.id(), new CANBus(rightId.bus()));
        homeInput = new DigitalInput(homeDio);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // Phoenix reports mechanism position in inches of carriage travel.
        config.Feedback.SensorToMechanismRatio = 1.0 / ElevatorK.IN_PER_MOTOR_ROT;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorK.CRUISE_IN_PER_S;
        config.MotionMagic.MotionMagicAcceleration = ElevatorK.MAX_ACCEL_IN_PER_S2;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.Slot0.kG = KG;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kP = KP;
        config.Slot0.kD = KD;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorK.SOFT_MAX_IN;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        // Soft limits are meaningless until the position is zeroed on the home switch.
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        left.getConfigurator().apply(config);

        // Right motor is mounted mirrored, so it opposes the leader.
        right.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(config.MotorOutput));
        right.setControl(new Follower(left.getDeviceID(), MotorAlignmentValue.Opposed));

        position = left.getPosition();
        velocity = left.getVelocity();
        appliedVolts = left.getMotorVoltage();
        statorAmps = left.getStatorCurrent();
        supplyAmps = left.getSupplyCurrent();
        followerPosition = right.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(FAST_SIGNAL_HZ, position, velocity);
        BaseStatusSignal.setUpdateFrequencyForAll(SLOW_SIGNAL_HZ, appliedVolts, statorAmps, supplyAmps);
        BaseStatusSignal.setUpdateFrequencyForAll(20, followerPosition);
        // Consume the reset flag from before this code started, so later reports mean a reboot while running.
        left.hasResetOccurred();
    }

    /** True while the carriage is on the home switch. Overridden by the simulation. */
    protected boolean homeSwitchClosed() {
        // Normally-open switch to ground: the roboRIO input is pulled up and reads false when closed.
        return !homeInput.get();
    }

    @Override
    public void updateInputs(ElevatorIOInputs in) {
        in.connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorAmps, supplyAmps).isOK();
        in.followerConnected = BaseStatusSignal.refreshAll(followerPosition).isOK();
        in.homeSwitch = homeSwitchClosed();
        in.carriageIn = position.getValueAsDouble();
        in.velocityInPerS = velocity.getValueAsDouble();
        in.appliedVolts = appliedVolts.getValueAsDouble();
        in.statorAmps = statorAmps.getValueAsDouble();
        in.supplyAmps = supplyAmps.getValueAsDouble();
        in.resetOccurred = left.hasResetOccurred();
    }

    @Override
    public void setTargetIn(double carriageIn) {
        left.setControl(positionRequest.withPosition(carriageIn));
    }

    @Override
    public void setVolts(double volts) {
        // Open-loop voltage is only used to find home, so the soft limits must not be in the way.
        setSoftLimits(false);
        left.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void zero() {
        left.setPosition(0);
        setSoftLimits(true);
    }

    @Override
    public void keepZero() {
        setSoftLimits(true);
    }

    @Override
    public void stop() {
        left.setControl(neutralRequest);
    }

    private void setSoftLimits(boolean enabled) {
        if (enabled == softLimitsEnabled) {
            return;
        }
        softLimitsEnabled = enabled;
        left.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(enabled)
                        .withReverseSoftLimitEnable(enabled)
                        .withForwardSoftLimitThreshold(ElevatorK.SOFT_MAX_IN)
                        .withReverseSoftLimitThreshold(0));
    }

    @Override
    public void close() {
        homeInput.close();
        left.close();
        right.close();
    }
}
