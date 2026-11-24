package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.ClimberK;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The climb of drawings CL-000 and R-001, one step at a time while the driver holds the climb button:
 * mechanism to the climb pose, open the funnel, wind in a little to unload the pawls, lift them, pay
 * the fork out to level, pawls down. The driver then backs onto the cage, holds the button again, and
 * the winch winds in and stops; the ratchets hold the robot from there, disabled or not. A second
 * button lets an empty fork back down after a missed cage.
 *
 * <p>Letting go at any point stops the winch with the pawls down. Opening the funnel is one way: from
 * then on {@link #committed()} keeps the mechanism in the climb pose, across code restarts too, until
 * {@link #pitReset()} or the next match begins with the fork stowed.
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
        /** Fork out, pawls down, waiting for the cage. */
        DEPLOYED,
        WINDING,
        CLIMBED,
        /** The fork's angle is unknown: the winch does nothing until the pit reset. */
        LOST
    }

    /** Below this at start-up, the fork is out of stow whatever else is known. */
    private static final double OUT_OF_STOW_DEG = 80;

    /**
     * A committed climber that reads this or more at start-up has lost its count: a climb ends at 88, and
     * a motor controller that lost power reads exactly stowed.
     */
    private static final double LOST_COUNT_DEG = 89.5;

    /** Stowed to within this is stowed, for starting the next match. */
    private static final double STOWED_TOL_DEG = 3;

    private static final double NOT_ANSWERING_S = 0.5;

    /** What the climber needs to know about the rest of the mechanism. */
    public record Mechanism(
            /** In the climb pose itself: the latch may fire. */
            BooleanSupplier atClimbPose,
            /** Known to be out of the fork's way. */
            BooleanSupplier clear,
            /** Known, from trusted sensors, to be in the fork's way. */
            BooleanSupplier inTheWay) {}

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final Mechanism mechanism;
    private final CommitMemory memory;
    private final Debouncer notAnswering = new Debouncer(NOT_ANSWERING_S);
    private final Alert lostAlert =
            new Alert("Climber does not know where its fork is: winch stopped, needs the pit reset", AlertType.kError);
    private final Alert usedAlert = new Alert(
            "Climb already begun: stow the fork, latch the tray, then press Climber/Pit reset", AlertType.kWarning);
    private final Alert stallAlert = new Alert("Climber winch stalled", AlertType.kError);
    private final Alert loadedAlert = new Alert("Fork is loaded: it will not be lowered", AlertType.kWarning);
    private final Alert motorAlert = new Alert("Climber motor not answering", AlertType.kError);

    private State state = State.STOWED;
    private boolean startDecided = false;
    private boolean committed = false;
    /** The commit was found at start-up, not made since. */
    private boolean restored = false;
    /** The latch is held open only by the code that opened it: a restart does not fire it again. */
    private boolean latchOpen = false;

    private boolean climbHeld = false;
    private boolean lowerHeld = false;
    /** The fork has been out: the climb button now means climb, not deploy. */
    private boolean deployedOnce = false;
    /** Both buttons have been up since the last step ended, so a press means the next one. */
    private boolean armed = false;

    private boolean pulling = false;
    private double climbHeldSinceS = Double.NaN;
    private double lastForkDeg = ClimberK.STOWED_DEG;
    private double stateStartS = 0;
    private double tensionTargetDeg;
    private State beforeTensioning = State.OPENING;
    /** A held load is a steady current one way; the position loop's ripple averages out. */
    private final LinearFilter ampsFilter = LinearFilter.movingAverage(10);

    private double meanAmps = 0;
    private double loadSinceS = Double.NaN;
    private double quietSinceS = Double.NaN;
    private double stalledSinceS = Double.NaN;
    private boolean stalled = false;
    private boolean refused = false;
    private double payOutFromDeg = ClimberK.STOWED_DEG;
    private int jamTries = ClimberK.JAM_TRIES;

    public Climber(ClimberIO io, Mechanism mechanism, CommitMemory memory) {
        this.io = io;
        this.mechanism = mechanism;
        this.memory = memory;
    }

    /** Whether the driver is holding the climb button. */
    public void setHeld(boolean held) {
        climbHeld = held;
    }

    /** Whether the driver is holding the button that lets an empty fork back down. */
    public void setLowerHeld(boolean held) {
        lowerHeld = held;
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

    /**
     * Someone in the pit has stowed the fork and latched the tray: forget the climb and count from stowed.
     * Does nothing while enabled.
     */
    public void pitReset() {
        if (DriverStation.isEnabled()) {
            return;
        }
        forgetClimb();
        io.zeroStowed();
    }

    private void forgetClimb() {
        memory.save("");
        committed = false;
        restored = false;
        latchOpen = false;
        deployedOnce = false;
        armed = false;
        lostAlert.set(false);
        usedAlert.set(false);
        enter(State.STOWED);
    }

    /** Release the alerts so a discarded climber does not leave them raised. */
    public void close() {
        lostAlert.close();
        usedAlert.close();
        stallAlert.close();
        loadedAlert.close();
        motorAlert.close();
    }

    private void enter(State next) {
        if (next != state) {
            state = next;
            stateStartS = Timer.getFPGATimestamp();
            stalledSinceS = Double.NaN;
            loadSinceS = Double.NaN;
            quietSinceS = Double.NaN;
        }
    }

    private double inState() {
        return Timer.getFPGATimestamp() - stateStartS;
    }

    private void lose() {
        lostAlert.set(true);
        enter(State.LOST);
    }

    /** Work out, once, what a restart of robot code has come back to. */
    private void decideStart() {
        if (memory.load().isEmpty()) {
            if (inputs.forkDeg < OUT_OF_STOW_DEG) {
                lose();
            }
            return;
        }
        committed = true;
        restored = true;
        deployedOnce = true;
        usedAlert.set(true);
        if (inputs.forkDeg >= LOST_COUNT_DEG) {
            lose();
        } else if (inputs.forkDeg >= OUT_OF_STOW_DEG) {
            enter(State.CLIMBED);
        } else {
            enter(State.DEPLOYED);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        double now = Timer.getFPGATimestamp();
        meanAmps = ampsFilter.calculate(inputs.statorAmps);

        if (!startDecided && inputs.connected) {
            startDecided = true;
            decideStart();
        }
        if (restored
                && DriverStation.isDisabled()
                && !memory.currentMatch().equals(CommitMemory.PRACTICE)
                && !memory.currentMatch().equals(memory.load())
                && Math.abs(inputs.forkDeg - ClimberK.STOWED_DEG) <= STOWED_TOL_DEG) {
            forgetClimb(); // another match, and the fork is back in stow
        }
        if (inputs.resetOccurred) {
            onMotorReboot();
        }
        motorAlert.set(notAnswering.calculate(startDecided && !(inputs.connected && inputs.followerConnected)));

        boolean enabled = DriverStation.isEnabled();
        boolean go = climbHeld && enabled;
        boolean lower = lowerHeld && enabled && !climbHeld;
        if (!go) {
            climbHeldSinceS = Double.NaN;
            pulling = false;
        } else if (Double.isNaN(climbHeldSinceS)) {
            climbHeldSinceS = now;
        }
        double heldS = go ? now - climbHeldSinceS : 0;
        if (!go && !lower) {
            jamTries = ClimberK.JAM_TRIES;
            armed = true;
            stalled = false;
            refused = false;
            stallAlert.set(false);
            loadedAlert.set(false);
        }
        boolean clear = mechanism.clear().getAsBoolean();
        // While deploying the climb button carries on; once the fork has been out, only the lower button does.
        boolean payOut = (deployedOnce ? lower : go) && clear && inputs.connected && !refused;

        boolean drive = false;
        boolean lift = false;
        double targetDeg = inputs.forkDeg;
        switch (state) {
            case STOWED -> {
                if (go && armed) {
                    enter(State.PREPARING);
                }
            }
            case PREPARING -> {
                if (!go) {
                    enter(State.STOWED);
                } else if (mechanism.atClimbPose().getAsBoolean() && inState() >= ClimberK.COMMIT_HOLD_S) {
                    committed = true;
                    latchOpen = true;
                    memory.save(memory.currentMatch());
                    enter(State.OPENING);
                }
            }
            case OPENING -> {
                if (!enabled) {
                    stateStartS = now; // the servo is off while disabled: give it its full time again
                } else if (inState() >= ClimberK.FUNNEL_OPEN_S && payOut) {
                    startTensioning();
                }
            }
            case TENSIONING -> {
                drive = payOut;
                targetDeg = tensionTargetDeg;
                boolean loaded = loadFelt(drive, now);
                boolean quiet = quietFor(drive && inputs.forkDeg >= tensionTargetDeg - ClimberK.TENSION_TOL_DEG, now);
                if (windRequested(go)) {
                    enter(State.WINDING);
                    drive = false;
                } else if (loaded) {
                    if (deployedOnce) {
                        refuse();
                    } else {
                        tensionTargetDeg = inputs.forkDeg; // on the stop bolts: that is tight enough
                        enter(State.LIFTING);
                    }
                    drive = false;
                } else if (quiet) {
                    // Held there by the motors with next to no current: nothing is hanging on the rope.
                    enter(State.LIFTING);
                }
            }
            case LIFTING -> {
                drive = payOut;
                lift = drive;
                targetDeg = tensionTargetDeg;
                if (!drive) {
                    restartTensioning();
                } else if (loadFelt(true, now)) {
                    drive = false;
                    lift = false;
                    refuse();
                } else if (inState() >= ClimberK.PAWL_LIFT_S) {
                    payOutFromDeg = inputs.forkDeg;
                    enter(State.PAYING_OUT);
                }
            }
            case PAYING_OUT -> {
                drive = payOut && !stalled;
                lift = drive;
                targetDeg = ClimberK.LEVEL_DEG;
                if (inputs.connected && Math.abs(inputs.forkDeg - ClimberK.LEVEL_DEG) <= ClimberK.LEVEL_TOL_DEG) {
                    drive = false;
                    lift = false;
                    armed = false;
                    deployedOnce = true;
                    enter(State.DEPLOYED);
                } else if (!payOut) {
                    // The pawls fall on the ratchet: unload them again before paying out on.
                    restartTensioning();
                } else if (inState() >= ClimberK.JAM_S && inputs.forkDeg > payOutFromDeg - ClimberK.JAM_DEG && jamTries > 0) {
                    // A pawl had just dropped over a tooth. The fork rests on that tooth now, so half a
                    // tooth from here is clear of both.
                    jamTries--;
                    drive = false;
                    lift = false;
                    restartTensioning();
                }
            }
            case DEPLOYED -> {
                if (windRequested(go) && heldS >= ClimberK.CLIMB_HOLD_S && clear) {
                    enter(State.WINDING);
                } else if (armed && payOut && inputs.forkDeg > ClimberK.LEVEL_DEG + ClimberK.TOOTH_DEG) {
                    startTensioning();
                }
            }
            case WINDING -> {
                // Checked when a pull starts. Under load the wrist's say is over: stopping half way helps
                // nobody, and an untrusted wrist is no reason to leave the robot part way up.
                if (go && !pulling && !mechanism.inTheWay().getAsBoolean()) {
                    pulling = true;
                }
                drive = pulling && !stalled && inputs.connected;
                targetDeg = ClimberK.CLIMBED_DEG;
                if (inputs.forkDeg >= ClimberK.CLIMBED_DEG - ClimberK.CLIMBED_TOL_DEG) {
                    drive = false;
                    armed = false;
                    enter(State.CLIMBED);
                } else if (armed && payOut) {
                    startTensioning();
                }
                if (go) {
                    armed = false;
                }
            }
            case CLIMBED -> {
                boolean settledBack =
                        inputs.forkDeg < ClimberK.CLIMBED_DEG - ClimberK.TOOTH_DEG - ClimberK.CLIMBED_TOL_DEG;
                if (windRequested(go) && settledBack) {
                    enter(State.WINDING);
                } else if (armed && payOut) {
                    startTensioning();
                }
            }
            case LOST -> {
                // Pawls down, motors off: the fork stays wherever it is.
            }
        }

        watchForStall(drive, now);
        if (drive && !stalled) {
            io.setTargetDeg(targetDeg);
        } else {
            io.stop();
        }
        io.setPawlsLifted(lift && !stalled);
        io.setFunnelOpen(latchOpen);
        lastForkDeg = inputs.forkDeg;

        Logger.recordOutput("Climber/State", state);
        Logger.recordOutput("Climber/Committed", committed);
        Logger.recordOutput("Climber/PawlsLifted", lift);
    }

    /** With the fork out, a fresh press of the climb button asks to wind in. */
    private boolean windRequested(boolean go) {
        return go && armed && deployedOnce;
    }

    private void onMotorReboot() {
        switch (state) {
            case STOWED, PREPARING, OPENING, LOST -> {
                // The count starts again at stowed, and the fork has not left it (or is already lost).
            }
            case TENSIONING, LIFTING -> {
                if (!deployedOnce && lastForkDeg >= ClimberK.STOWED_DEG - STOWED_TOL_DEG) {
                    enter(State.OPENING);
                } else {
                    lose();
                }
            }
            default -> lose();
        }
    }

    private void startTensioning() {
        beforeTensioning = state == State.OPENING ? State.OPENING : state;
        restartTensioning();
    }

    private void restartTensioning() {
        tensionTargetDeg = Math.min(inputs.forkDeg + ClimberK.TENSION_DEG, ClimberK.TENSION_MAX_DEG);
        enter(State.TENSIONING);
    }

    /** The robot is hanging on the rope, or something has hold of the fork: pawls down until the button is let go. */
    private void refuse() {
        refused = true;
        loadedAlert.set(true);
        if (deployedOnce) {
            enter(beforeTensioning);
        } else {
            restartTensioning();
        }
    }

    /** An unloaded fork takes a few amps to wind in; the stop bolts or the robot's weight take far more. */
    private boolean loadFelt(boolean driving, double now) {
        if (!driving || Math.abs(meanAmps) < ClimberK.TENSION_LOAD_A) {
            loadSinceS = Double.NaN;
            return false;
        }
        if (Double.isNaN(loadSinceS)) {
            loadSinceS = now;
        }
        return now - loadSinceS >= ClimberK.TENSION_LOAD_S;
    }

    /** True once the fork has been held in place for a while with next to no current. */
    private boolean quietFor(boolean holding, double now) {
        if (!holding || Math.abs(meanAmps) >= ClimberK.TENSION_LOAD_A) {
            quietSinceS = Double.NaN;
            return false;
        }
        if (Double.isNaN(quietSinceS)) {
            quietSinceS = now;
        }
        return now - quietSinceS >= ClimberK.TENSION_QUIET_S;
    }

    /** A winch at its current limit and not turning has hit something: stop until the button is released. */
    private void watchForStall(boolean drive, double now) {
        boolean pushing = drive
                && Math.abs(inputs.statorAmps) > ClimberK.STALL_A
                && Math.abs(inputs.velocityDegPerS) < ClimberK.STALL_DEG_PER_S;
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
