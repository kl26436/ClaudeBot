package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.config.Ports.CanId;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotConstants.ClimberK;

/**
 * Two Kraken X60s on the winch (left leads, right follows the other way), two pawl servos on one PWM
 * output, and the funnel latch servo.
 *
 * <p>Phoenix position is zero with the fork stowed, so a freshly powered controller is right as long
 * as the robot starts stowed, and a robot-code restart keeps the position.
 */
public class ClimberIOTalonFX implements ClimberIO, AutoCloseable {
    // Control gains in volts, per rotation of the fork.
    protected static final double KV = 19.4; // PLACEHOLDER(tuning)
    protected static final double KP = 720; // PLACEHOLDER(tuning)

    private static final double SIGNAL_HZ = 50;

    /** REV Smart Robot Servo, default range: 500 to 2500 microseconds over 270 degrees. */
    private static final double US_PER_DEG = 2000.0 / 270.0;

    // Each horn is fitted so 1500 us is the middle of its swing, and the ends are half a swing either
    // side. Which end is which, and the exact pulses, are found on the robot.
    private static final double PAWL_HALF_US = ClimberK.PAWL_HORN_SWING_DEG / 2 * US_PER_DEG;
    private static final double LATCH_HALF_US = ClimberK.LATCH_HORN_SWING_DEG / 2 * US_PER_DEG;
    private static final int PAWL_DOWN_US = (int) (1500 + PAWL_HALF_US); // PLACEHOLDER(robot-check)
    private static final int PAWL_LIFTED_US = (int) (1500 - PAWL_HALF_US); // PLACEHOLDER(robot-check)
    private static final int LATCH_CLOSED_US = (int) (1500 + LATCH_HALF_US); // PLACEHOLDER(robot-check)
    private static final int LATCH_OPEN_US = (int) (1500 - LATCH_HALF_US); // PLACEHOLDER(robot-check)

    protected final TalonFX left;
    protected final TalonFX right;
    private final Servo pawls;
    private final Servo latch;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorAmps;
    private final StatusSignal<Angle> followerPosition;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.PRO_LICENSED);
    private final NeutralOut neutralRequest = new NeutralOut();

    public ClimberIOTalonFX(CanId leftId, CanId rightId, int pawlPwm, int latchPwm) {
        left = new TalonFX(leftId.id(), new CANBus(leftId.bus()));
        right = new TalonFX(rightId.id(), new CANBus(rightId.bus()));
        pawls = new Servo(pawlPwm);
        latch = new Servo(latchPwm);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Positive winds the rope in. Check the direction with the pawls lifted before roping up.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // PLACEHOLDER(robot-check)
        config.Feedback.SensorToMechanismRatio = ClimberK.GEAR_RATIO;
        config.Slot0.kV = KV;
        config.Slot0.kP = KP;
        config.MotionMagic.MotionMagicCruiseVelocity = ClimberK.CRUISE_DEG_PER_S / 360.0;
        config.MotionMagic.MotionMagicAcceleration = ClimberK.ACCEL_DEG_PER_S2 / 360.0;
        config.CurrentLimits.StatorCurrentLimit = ClimberK.STATOR_LIMIT_A;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        left.getConfigurator().apply(config);
        right.getConfigurator()
                .apply(new TalonFXConfiguration()
                        .withMotorOutput(config.MotorOutput)
                        .withCurrentLimits(config.CurrentLimits));
        right.setControl(new Follower(left.getDeviceID(), MotorAlignmentValue.Opposed));

        position = left.getPosition();
        velocity = left.getVelocity();
        appliedVolts = left.getMotorVoltage();
        statorAmps = left.getStatorCurrent();
        followerPosition = right.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_HZ, position, velocity, appliedVolts, statorAmps);
        BaseStatusSignal.setUpdateFrequencyForAll(10, followerPosition);
        // Consume the reset flag from before this code started, so later reports mean a reboot while running.
        left.hasResetOccurred();
    }

    @Override
    public void updateInputs(ClimberIOInputs in) {
        in.connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorAmps).isOK();
        in.followerConnected = BaseStatusSignal.refreshAll(followerPosition).isOK();
        in.forkDeg = ClimberK.STOWED_DEG + position.getValueAsDouble() * 360.0;
        in.velocityDegPerS = velocity.getValueAsDouble() * 360.0;
        in.appliedVolts = appliedVolts.getValueAsDouble();
        in.statorAmps = statorAmps.getValueAsDouble();
        in.resetOccurred = left.hasResetOccurred();
    }

    @Override
    public void setTargetDeg(double forkDeg) {
        left.setControl(positionRequest.withPosition((forkDeg - ClimberK.STOWED_DEG) / 360.0));
    }

    @Override
    public void stop() {
        left.setControl(neutralRequest);
    }

    @Override
    public void zeroStowed() {
        left.setPosition(0);
    }

    @Override
    public void setPawlsLifted(boolean lifted) {
        pawls.setPulseTimeMicroseconds(lifted ? PAWL_LIFTED_US : PAWL_DOWN_US);
    }

    @Override
    public void setFunnelOpen(boolean open) {
        latch.setPulseTimeMicroseconds(open ? LATCH_OPEN_US : LATCH_CLOSED_US);
    }

    @Override
    public void close() {
        left.close();
        right.close();
        pawls.close();
        latch.close();
    }
}
