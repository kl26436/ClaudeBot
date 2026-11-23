package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.WristK;
import frc.robot.config.Setpoints;
import org.littletonrobotics.junction.Logger;

/**
 * Wrist joint. The motor controller closes its loop on its own rotor, seeded from the absolute
 * encoder on the pivot: at start-up, after the controller reboots, and when a little chain slack has
 * built up on a wrist that is holding still. It only moves while the encoder can be trusted:
 * connected, reading an angle the mechanism can physically reach, and agreeing with the motor. Until
 * a goal is set it holds in brake.
 */
public class Wrist extends SubsystemBase {
    /** Outside this range the encoder is reading something the hard stops make impossible. */
    private static final double TRUST_MIN_DEG = -95;

    private static final double TRUST_MAX_DEG = 80;

    /**
     * The motor's closed-loop angle and the absolute encoder must agree this closely. A lasting gap
     * means the chain has slipped or the encoder has moved on its shaft.
     */
    private static final double MAX_DISAGREEMENT_DEG = 5;

    /**
     * How long a disagreement must last to count. The motor's reading comes over CAN a few milliseconds
     * behind the encoder's, so mid-move the two can briefly differ by more than the limit.
     */
    private static final double DISAGREEMENT_DEBOUNCE_S = 0.1;

    /** A gap above this, on a wrist that has been still for {@link #STILL_S}, is taken out by seeding again. */
    private static final double RESEED_DEG = 0.5; // PLACEHOLDER(tuning)

    private static final double STILL_DEG_PER_S = 2;
    private static final double STILL_S = 0.25;

    private final Debouncer stillOffset = new Debouncer(STILL_S, DebounceType.kRising);
    private final Debouncer disagreement = new Debouncer(DISAGREEMENT_DEBOUNCE_S, DebounceType.kRising);
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final Alert encoderAlert = new Alert("Wrist encoder not trusted", AlertType.kError);

    private double goalDeg = Double.NaN;
    private boolean trusted = false;
    /** True from start-up or a motor reboot until the motor has taken the encoder's angle. */
    private boolean seeding = true;

    private boolean seedSent = false;

    public Wrist(WristIO io) {
        this.io = io;
    }

    /** Set the angle goal, clamped to the soft travel. */
    public void setGoalDeg(double deg) {
        goalDeg = MathUtil.clamp(deg, WristK.MIN_DEG, WristK.MAX_DEG);
    }

    public double wristDeg() {
        return inputs.wristDeg;
    }

    /** Measured wrist speed, degrees per second, positive up. */
    public double velocityDegPerS() {
        return inputs.velocityDegPerS;
    }

    public boolean atGoal() {
        return trusted && !Double.isNaN(goalDeg) && Math.abs(inputs.wristDeg - goalDeg) <= Setpoints.WRIST_TOL_DEG;
    }

    /** False when the absolute encoder is unplugged or reads an angle the wrist cannot be at. */
    public boolean isTrusted() {
        return trusted;
    }

    /** Release the alert so a discarded wrist does not leave it raised. */
    public void close() {
        encoderAlert.close();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        boolean encoderOk = inputs.encoderConnected
                && inputs.absoluteDeg >= TRUST_MIN_DEG
                && inputs.absoluteDeg <= TRUST_MAX_DEG;
        double gap = Math.abs(inputs.wristDeg - inputs.absoluteDeg);
        boolean agrees = gap <= MAX_DISAGREEMENT_DEG; // NaN disagrees
        if (inputs.resetOccurred) {
            seeding = true;
            seedSent = false;
        }
        if (seeding && seedSent && agrees) {
            seeding = false;
        }
        // Both are fed every loop, so the debounce timing does not depend on the other checks.
        boolean slack = stillOffset.calculate(
                agrees && gap > RESEED_DEG && Math.abs(inputs.velocityDegPerS) < STILL_DEG_PER_S);
        boolean disagrees = disagreement.calculate(!agrees);
        if (inputs.connected && encoderOk && (seeding || slack)) {
            io.seed(inputs.absoluteDeg);
            seedSent = true;
        }
        trusted = encoderOk && !seeding && !disagrees;
        encoderAlert.set(!trusted);

        if (trusted && !Double.isNaN(goalDeg)) {
            io.setTargetDeg(goalDeg);
        } else {
            io.stop();
        }

        Logger.recordOutput("Wrist/Trusted", trusted);
        Logger.recordOutput("Wrist/GoalDeg", goalDeg);
    }
}
