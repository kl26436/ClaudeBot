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
 * Wrist joint. It only moves while its absolute encoder can be trusted: connected, and reading an
 * angle the mechanism can physically reach. Until a goal is set it holds in brake.
 */
public class Wrist extends SubsystemBase {
    /** Outside this range the encoder is reading something the hard stops make impossible. */
    private static final double TRUST_MIN_DEG = -95;

    private static final double TRUST_MAX_DEG = 80;

    /**
     * The motor's closed-loop angle and the absolute encoder must agree this closely. A lasting gap
     * means the motor has lost its remote sensor and is closing the loop on a wrong angle.
     */
    private static final double MAX_DISAGREEMENT_DEG = 5;

    /**
     * How long a disagreement must last to count. The two readings arrive in separate frames a few
     * milliseconds apart, so mid-move under a busy bus they can briefly differ by more than the limit.
     */
    private static final double DISAGREEMENT_DEBOUNCE_S = 0.1;

    private final Debouncer disagreement = new Debouncer(DISAGREEMENT_DEBOUNCE_S, DebounceType.kRising);
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final Alert encoderAlert = new Alert("Wrist encoder not trusted", AlertType.kError);

    private double goalDeg = Double.NaN;
    private boolean trusted = false;

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

        // Fed every loop, so the debounce timing does not depend on the other checks.
        boolean disagrees = disagreement.calculate(
                !(Math.abs(inputs.wristDeg - inputs.absoluteDeg) <= MAX_DISAGREEMENT_DEG)); // NaN disagrees
        trusted = inputs.encoderConnected
                && inputs.absoluteDeg >= TRUST_MIN_DEG
                && inputs.absoluteDeg <= TRUST_MAX_DEG
                && !disagrees;
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
