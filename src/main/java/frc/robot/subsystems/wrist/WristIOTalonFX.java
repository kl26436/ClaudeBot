package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.config.Ports.CanId;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.WristK;

/**
 * One Kraken X60 on the wrist, closing its loop on its own rotor, and the REV Through Bore encoder on
 * the pivot, read by the roboRIO as a duty cycle. Phoenix works in rotations of the wrist with zero
 * at level, which is what the arm-cosine gravity term needs; this class converts to and from degrees.
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

    /** +1 if the encoder reading rises as the tip rises, -1 if it falls. */
    protected static final double ENCODER_SIGN = 1.0; // PLACEHOLDER(tuning)

    /** A Through Bore encoder's absolute output is 1 to 1024 microseconds high in a 1025 microsecond frame. */
    private static final double ENCODER_FRAME_US = 1025;

    private static final double ENCODER_HZ = 1e6 / ENCODER_FRAME_US;

    private static final double FAST_SIGNAL_HZ = 250;
    private static final double SLOW_SIGNAL_HZ = 50;

    protected final TalonFX motor;
    protected final DutyCycleEncoder encoder;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorAmps;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.PRO_LICENSED);
    private final NeutralOut neutralRequest = new NeutralOut();

    public WristIOTalonFX(CanId motorId, int encoderDio) {
        motor = new TalonFX(motorId.id(), new CANBus(motorId.bus()));
        encoder = new DutyCycleEncoder(encoderDio);
        encoder.setAssumedFrequency(ENCODER_HZ);
        encoder.setDutyCycleRange(1.0 / ENCODER_FRAME_US, 1024.0 / ENCODER_FRAME_US);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = WristK.GEAR_RATIO;
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
        BaseStatusSignal.setUpdateFrequencyForAll(FAST_SIGNAL_HZ, position, velocity);
        BaseStatusSignal.setUpdateFrequencyForAll(SLOW_SIGNAL_HZ, appliedVolts, statorAmps);
        // Switch off every signal not set above, to keep the bus (and the simulator) light.
        ParentDevice.optimizeBusUtilizationForAll(motor);
        // Consume the reset flag from before this code started, so later reports mean a reboot while running.
        motor.hasResetOccurred();
    }

    @Override
    public void updateInputs(WristIOInputs in) {
        in.connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorAmps).isOK();
        in.encoderConnected = encoder.isConnected();
        in.wristDeg = position.getValueAsDouble() * 360.0;
        // Wrapped to -0.5..0.5 rotations, so the whole -90 to +75 degree travel has no jump in it.
        in.absoluteDeg = MathUtil.inputModulus(ENCODER_SIGN * (encoder.get() - ENCODER_OFFSET_ROT), -0.5, 0.5) * 360.0;
        in.velocityDegPerS = velocity.getValueAsDouble() * 360.0;
        in.appliedVolts = appliedVolts.getValueAsDouble();
        in.statorAmps = statorAmps.getValueAsDouble();
        in.resetOccurred = motor.hasResetOccurred();
    }

    @Override
    public void seed(double deg) {
        motor.setPosition(deg / 360.0, 0); // no wait: the next loop's reading shows whether it took
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
        encoder.close();
    }
}
