package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.ClimberK;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The climb of drawings CL-000 and R-001, one step at a time while the driver holds the button:
 * mechanism to the climb pose, open the funnel, wind in a little to unload the pawls, lift them, pay
 * the fork out to level, pawls down. The driver then backs onto the cage, presses again, and the winch
 * winds in and stops; the ratchets hold the robot from there, disabled or not.
 *
 * <p>Letting go at any point stops the winch with the pawls down. Opening the funnel is one way: from
 * then on {@link #committed()} keeps the mechanism in the climb pose until robot code restarts in the
 * pit.
 */
public class Climber extends SubsystemBase {
    public enum State {
        STOWED,
        /** Waiting for the mechanism to reach the climb pose. */
        PREPARING,
        /** Latch released: the tubing is throwing the tray open. */
        OPENING,
        /** Winding in a little so the pawls carry no load. */
        TENSIONING,
        LIFTING,
        PAYING_OUT,
        /** Fork level, pawls down, waiting for the cage. */
        DEPLOYED,
        WINDING,
        CLIMBED,
        /** The motor controller rebooted with the fork out: its angle is unknown. */
        LOST
    }

    /** Below this at start-up, robot code has restarted with the fork already out. */
    private static final double OUT_OF_STOW_DEG = 80;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final BooleanSupplier atClimbPose;
    private final BooleanSupplier clearToTurn;
    private final Alert lostAlert = new Alert("Climber lost its position: winch stopped", AlertType.kError);
    private final Alert stallAlert = new Alert("Climber winch stalled", AlertType.kError);

    private State state = State.STOWED;
    private boolean startDecided = false;
    private boolean committed = false;
    private boolean held = false;
    /** The button has been up since the fork reached level, so a press now means climb. */
    private boolean armed = false;

    private double stateStartS = 0;
    private double tensionTargetDeg;
    private double stalledSinceS = Double.NaN;
    private boolean stalled = false;

    /**
     * @param atClimbPose the mechanism has reached the climb pose: the latch may fire
     * @param clearToTurn the mechanism is still out of the fork's way: the fork may move
     */
    public Climber(ClimberIO io, BooleanSupplier atClimbPose, BooleanSupplier clearToTurn) {
        this.io = io;
        this.atClimbPose = atClimbPose;
        this.clearToTurn = clearToTurn;
    }

    /** Whether the driver is holding the climb button. */
    public void setHeld(boolean held) {
        this.held = held;
    }

    public State state() {
        return state;
    }

    /** True once the funnel has been opened: the mechanism stays in the climb pose. */
    public boolean committed() {
        return committed;
    }

    public double forkDeg() {
        return inputs.forkDeg;
    }

    /** Release the alerts so a discarded climber does not leave them raised. */
    public void close() {
        lostAlert.close();
        stallAlert.close();
    }

    private void enter(State next) {
        if (next != state) {
            state = next;
            stateStartS = Timer.getFPGATimestamp();
            stalledSinceS = Double.NaN;
        }
    }

    private double inState() {
        return Timer.getFPGATimestamp() - stateStartS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (!startDecided && inputs.connected) {
            startDecided = true;
            if (inputs.forkDeg < OUT_OF_STOW_DEG) {
                committed = true;
                enter(State.DEPLOYED);
            }
        }
        if (inputs.resetOccurred && state != State.STOWED && state != State.PREPARING && state != State.OPENING) {
            lostAlert.set(true);
            enter(State.LOST);
        }
        boolean go = held && DriverStation.isEnabled();
        if (!go) {
            stalled = false;
            stallAlert.set(false);
        }

        boolean drive = false;
        boolean lift = false;
        double targetDeg = inputs.forkDeg;
        switch (state) {
            case STOWED -> {
                if (go) {
                    enter(State.PREPARING);
                }
            }
            case PREPARING -> {
                if (!go) {
                    enter(State.STOWED);
                } else if (atClimbPose.getAsBoolean()) {
                    committed = true;
                    enter(State.OPENING);
                }
            }
            case OPENING -> {
                if (inState() >= ClimberK.FUNNEL_OPEN_S && go && clearToTurn.getAsBoolean()) {
                    startTensioning();
                }
            }
            case TENSIONING -> {
                drive = go && clearToTurn.getAsBoolean();
                targetDeg = tensionTargetDeg;
                if (!drive) {
                    stateStartS = Timer.getFPGATimestamp();
                } else if (inputs.forkDeg >= tensionTargetDeg - ClimberK.TENSION_TOL_DEG) {
                    enter(State.LIFTING);
                }
            }
            case LIFTING -> {
                drive = go && clearToTurn.getAsBoolean();
                lift = drive;
                targetDeg = tensionTargetDeg;
                if (!drive) {
                    startTensioning();
                } else if (inState() >= ClimberK.PAWL_LIFT_S) {
                    enter(State.PAYING_OUT);
                }
            }
            case PAYING_OUT -> {
                drive = go && clearToTurn.getAsBoolean();
                lift = drive;
                targetDeg = ClimberK.LEVEL_DEG;
                if (Math.abs(inputs.forkDeg - ClimberK.LEVEL_DEG) <= ClimberK.LEVEL_TOL_DEG) {
                    armed = false;
                    drive = false;
                    lift = false;
                    enter(State.DEPLOYED);
                } else if (!drive) {
                    startTensioning(); // the pawls fall on the ratchet: unload them again before paying out on
                }
            }
            case DEPLOYED -> {
                armed |= !go;
                if (armed && go && clearToTurn.getAsBoolean()) {
                    enter(State.WINDING);
                }
            }
            case WINDING -> {
                // Under load the wrist's say is over: stopping half way helps nobody, and the pawls are down.
                drive = go && !stalled;
                targetDeg = ClimberK.CLIMBED_DEG;
                if (inputs.forkDeg >= ClimberK.CLIMBED_DEG - ClimberK.CLIMBED_TOL_DEG) {
                    drive = false;
                    armed = false;
                    enter(State.CLIMBED);
                }
            }
            case CLIMBED -> {
                armed |= !go;
                boolean settledBack = inputs.forkDeg < ClimberK.CLIMBED_DEG - ClimberK.TOOTH_DEG - ClimberK.CLIMBED_TOL_DEG;
                if (armed && go && settledBack) {
                    enter(State.WINDING);
                }
            }
            case LOST -> {
                // Pawls down, motors off: the fork stays wherever it is.
            }
        }

        watchForStall(drive);
        if (drive && !stalled) {
            io.setTargetDeg(targetDeg);
        } else {
            io.stop();
        }
        io.setPawlsLifted(lift);
        io.setFunnelOpen(committed);

        Logger.recordOutput("Climber/State", state);
        Logger.recordOutput("Climber/Committed", committed);
        Logger.recordOutput("Climber/PawlsLifted", lift);
    }

    private void startTensioning() {
        tensionTargetDeg = Math.min(inputs.forkDeg + ClimberK.TENSION_DEG, ClimberK.TENSION_MAX_DEG);
        enter(State.TENSIONING);
    }

    /** A winch at its current limit and not turning has hit something: stop until the button is released. */
    private void watchForStall(boolean drive) {
        boolean pushing = drive
                && Math.abs(inputs.statorAmps) > ClimberK.STALL_A
                && Math.abs(inputs.velocityDegPerS) < ClimberK.STALL_DEG_PER_S;
        double now = Timer.getFPGATimestamp();
        if (!pushing) {
            stalledSinceS = Double.NaN;
        } else if (Double.isNaN(stalledSinceS)) {
            stalledSinceS = now;
        } else if (now - stalledSinceS >= ClimberK.STALL_S) {
            stalled = true;
            stallAlert.set(true);
        }
    }
}
