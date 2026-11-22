package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.Setpoints;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator carriage. At start-up it decides once whether the motor controller still holds a zero:
 * the Talon keeps its position across a robot-code restart, and an unpowered carriage falls to the
 * bottom, so a reading above {@link #KEPT_ZERO_MIN_IN} is a zero from before the restart and is kept
 * where it is. Otherwise, on first enable, it drives down at {@link ElevatorK#HOME_VOLTS} until the
 * home switch closes (or it stalls), then zeroes. Goals are ignored until it is homed.
 *
 * <p>If the motor controller reboots while running (a brown-out), the position is lost: the carriage
 * stops, an alert asks for it to be lowered by hand, and nothing moves it again until robot code
 * restarts. Driving down blind could take the wrist through the funnel band.
 */
public class Elevator extends SubsystemBase {
    private enum State {
        UNHOMED,
        HOMING,
        HOMED,
        LOST
    }

    /** A start-up reading above this is a zero kept from before a code restart. */
    static final double KEPT_ZERO_MIN_IN = 1.0;

    /** Below this speed the carriage counts as stalled while homing. */
    private static final double STALL_SPEED_IN_PER_S = 0.5;

    private static final double STALL_TIME_S = 0.25;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Alert homeSwitchAlert = new Alert("Elevator home switch not seen", AlertType.kWarning);
    private final Alert lostAlert = new Alert(
            "Elevator lost its position: lower it by hand, then restart the robot code", AlertType.kError);
    private boolean startDecided = false;

    private State state = State.UNHOMED;
    private double goalIn = 0;
    private double homingStartS;
    private double stalledSinceS = Double.NaN;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    /** Set the carriage goal, clamped to the travel. Ignored until the elevator is homed. */
    public void setGoalIn(double carriageIn) {
        if (state != State.HOMED) {
            return;
        }
        goalIn = MathUtil.clamp(carriageIn, 0, ElevatorK.SOFT_MAX_IN);
    }

    public double carriageIn() {
        return inputs.carriageIn;
    }

    public double velocityInPerS() {
        return inputs.velocityInPerS;
    }

    public boolean atGoal() {
        return state == State.HOMED && Math.abs(inputs.carriageIn - goalIn) <= Setpoints.CARRIAGE_TOL_IN;
    }

    public boolean isHomed() {
        return state == State.HOMED;
    }

    /** Forget the zero and run the homing routine again on the next enabled cycle. */
    public void requestHome() {
        if (state == State.LOST) {
            return; // only a code restart, after the carriage is lowered by hand, clears a lost position
        }
        state = State.UNHOMED;
        goalIn = 0;
    }

    /** Release the alert so a discarded elevator does not leave it raised. */
    public void close() {
        homeSwitchAlert.close();
        lostAlert.close();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (!startDecided && inputs.connected) {
            startDecided = true;
            // On the switch, just zero there; above it, a reading is a zero kept from before a restart.
            if (state == State.UNHOMED && !inputs.homeSwitch && inputs.carriageIn > KEPT_ZERO_MIN_IN) {
                io.keepZero();
                goalIn = inputs.carriageIn;
                state = State.HOMED;
                Logger.recordOutput("Elevator/KeptZeroIn", inputs.carriageIn);
            }
        }
        if (state == State.HOMED && inputs.resetOccurred) {
            state = State.LOST;
            lostAlert.set(true);
        }

        if (state == State.UNHOMED && startDecided && DriverStation.isEnabled()) {
            state = State.HOMING;
            homingStartS = Timer.getFPGATimestamp();
            stalledSinceS = Double.NaN;
        }

        switch (state) {
            case UNHOMED, LOST -> io.stop();
            case HOMING -> home();
            case HOMED -> io.setTargetIn(goalIn);
        }

        Logger.recordOutput("Elevator/State", state);
        Logger.recordOutput("Elevator/GoalIn", goalIn);
    }

    private void home() {
        if (!DriverStation.isEnabled()) {
            // Disabled mid-homing: the motors are off, so start the stall timing over.
            stalledSinceS = Double.NaN;
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (inputs.homeSwitch) {
            finishHoming();
            return;
        }
        if (Math.abs(inputs.velocityInPerS) < STALL_SPEED_IN_PER_S) {
            if (Double.isNaN(stalledSinceS)) {
                stalledSinceS = now;
            }
        } else {
            stalledSinceS = Double.NaN;
        }
        boolean timedOut = now - homingStartS >= ElevatorK.HOME_TIMEOUT_S;
        if (timedOut && !Double.isNaN(stalledSinceS) && now - stalledSinceS >= STALL_TIME_S) {
            homeSwitchAlert.set(true);
            finishHoming();
            return;
        }
        io.setVolts(ElevatorK.HOME_VOLTS);
    }

    private void finishHoming() {
        io.zero();
        io.stop();
        goalIn = 0;
        state = State.HOMED;
    }
}
