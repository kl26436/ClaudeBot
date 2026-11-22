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
 * Elevator carriage. Un-homed at boot: on first enable it drives down at {@link ElevatorK#HOME_VOLTS}
 * until the home switch closes, then zeroes. Goals are ignored until it is homed.
 */
public class Elevator extends SubsystemBase {
    private enum State {
        UNHOMED,
        HOMING,
        HOMED
    }

    /** Below this speed the carriage counts as stalled while homing. */
    private static final double STALL_SPEED_IN_PER_S = 0.5;

    private static final double STALL_TIME_S = 0.25;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Alert homeSwitchAlert = new Alert("Elevator home switch not seen", AlertType.kWarning);

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
        state = State.UNHOMED;
        goalIn = 0;
    }

    /** Release the alert so a discarded elevator does not leave it raised. */
    public void close() {
        homeSwitchAlert.close();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (state == State.UNHOMED && DriverStation.isEnabled()) {
            state = State.HOMING;
            homingStartS = Timer.getFPGATimestamp();
            stalledSinceS = Double.NaN;
        }

        switch (state) {
            case UNHOMED -> io.stop();
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
