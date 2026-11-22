package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.config.Ports.CanId;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.WristK;

/**
 * One Kraken X60 on the wrist, closing its loop on the REV Through Bore encoder that a CANdi reads
 * on its PWM input 1. Phoenix works in rotations of the wrist with zero at level, which is what the
 * arm-cosine gravity term needs; this class converts to and from degrees.
 */
public class WristIOTalonFX implements WristIO, AutoCloseable {
    // Control gains in volts, per rotation of the wrist.
    protected static final double KG = 0.30; // PLACEHOLDER(tuning)
    protected static final double KS = 0.0; // PLACEHOLDER(tuning)
    protected static final double KV = 7.45; // PLACEHOLDER(tuning)
    protected static final double KA = 0.05; // PLACEHOLDER(tuning)
    protected static final double KP = 60.0; // PLACEHOLDER(tuning)
    protected static final double KD = 1.0; // PLACEHOLDER(tuning)

    /** Encoder reading, in rotations, when the wrist is level. Found once on the built robot. */
    protected static final double ENCODER_OFFSET_ROT = 0.0; // PLACEHOLDER(tuning)

    /**
     * Phoenix reports the PWM frame length in microseconds. A Through Bore encoder's frame is about
     * 1025 us long; a missing signal reads as zero.
     */
    private static final double MIN_PWM_PERIOD_US = 500;

    private static final double MAX_PWM_PERIOD_US = 2000;

    private static final double FAST_SIGNAL_HZ = 250;
    private static final double SLOW_SIGNAL_HZ = 50;

    protected final TalonFX motor;
    protected final CANdi candi;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorAmps;
    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Time> pwmPeriod;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.PRO_LICENSED);
    private final NeutralOut neutralRequest = new NeutralOut();

    public WristIOTalonFX(CanId motorId, CanId candiId) {
        motor = new TalonFX(motorId.id(), new CANBus(motorId.bus()));
        candi = new CANdi(candiId.id(), new CANBus(candiId.bus()));

        var candiConfig = new CANdiConfiguration();
        candiConfig.PWM1.AbsoluteSensorOffset = ENCODER_OFFSET_ROT;
        // Report -0.5 to +0.5 rotations, so the whole -90 to +75 degree travel has no wrap in it.
        candiConfig.PWM1.AbsoluteSensorDiscontinuityPoint = 0.5;
        candi.getConfigurator().apply(candiConfig);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackRemoteSensorID = candiId.id();
        // Fusing the encoder with the rotor is a Phoenix Pro feature; without it, use the encoder alone.
        config.Feedback.FeedbackSensorSource = RobotConstants.PRO_LICENSED
                ? FeedbackSensorSourceValue.FusedCANdiPWM1
                : FeedbackSensorSourceValue.RemoteCANdiPWM1;
        config.Feedback.RotorToSensorRatio = WristK.GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = KG;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kP = KP;
        config.Slot0.kD = KD;
        config.MotionMagic.MotionMagicCruiseVelocity = WristK.CRUISE_DEG_PER_S / 360.0;
        config.MotionMagic.MotionMagicAcceleration = WristK.ACCEL_DEG_PER_S2 / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristK.MAX_DEG / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristK.MIN_DEG / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = WristK.STATOR_LIMIT_A;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        statorAmps = motor.getStatorCurrent();
        absolutePosition = candi.getPWM1Position();
        pwmPeriod = candi.getPWM1RiseToRise();
        BaseStatusSignal.setUpdateFrequencyForAll(FAST_SIGNAL_HZ, position, velocity);
        BaseStatusSignal.setUpdateFrequencyForAll(SLOW_SIGNAL_HZ, appliedVolts, statorAmps, absolutePosition, pwmPeriod);
    }

    @Override
    public void updateInputs(WristIOInputs in) {
        in.connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorAmps).isOK();
        boolean candiAnswering = BaseStatusSignal.refreshAll(absolutePosition, pwmPeriod).isOK();
        double periodUs = pwmPeriod.getValueAsDouble();
        in.encoderConnected = candiAnswering && periodUs >= MIN_PWM_PERIOD_US && periodUs <= MAX_PWM_PERIOD_US;
        in.wristDeg = position.getValueAsDouble() * 360.0;
        in.absoluteDeg = absolutePosition.getValueAsDouble() * 360.0;
        in.velocityDegPerS = velocity.getValueAsDouble() * 360.0;
        in.appliedVolts = appliedVolts.getValueAsDouble();
        in.statorAmps = statorAmps.getValueAsDouble();
    }

    @Override
    public void setTargetDeg(double deg) {
        motor.setControl(positionRequest.withPosition(deg / 360.0));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }

    @Override
    public void close() {
        motor.close();
        candi.close();
    }
}
