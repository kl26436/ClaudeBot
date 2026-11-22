package frc.robot.testutil;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;

/**
 * Steps the HAL, Phoenix and the command scheduler in 20 ms real-time ticks, with the registered
 * physics steps run three times per tick.
 *
 * <p>Phoenix 6 simulated devices run on their own real-time thread, so the harness cannot skip
 * ahead: each tick sleeps to its deadline. Tests call {@link #reset()} before and after each run so
 * that registered steps and scheduler state do not leak between tests.
 */
public final class SimHarness {
    /** Scheduler period in seconds. Physics steps take {@link #lastDtSeconds()}, not this. */
    public static final double DT = 0.02;

    /**
     * Physics steps per scheduler tick. Phoenix closes its loops at 1 kHz on whatever position the
     * physics last wrote, so a position that only moves every 20 ms looks like a sawtooth error to
     * it; with a 0.025 ohm motor that is tens of amps of current that no real robot would draw.
     * Stepping the physics every 5 ms keeps that ripple small.
     */
    private static final int PHYSICS_STEPS_PER_TICK = 3;

    private static final long PHYSICS_STEP_NS = 5_000_000;

    private static final int PHOENIX_ENABLE_TIMEOUT_MS = 100;

    /**
     * Real time the simulated devices get to publish what a sim step just wrote before the
     * scheduler reads their signals. Without it, a read races the Phoenix thread and repeats or
     * skips a sample at random, which shows up as spikes in any derived quantity.
     */
    private static final long PHOENIX_SETTLE_MS = 6;

    private static final List<Runnable> steps = new ArrayList<>();
    private static boolean halInitialised = false;
    private static long lastTickNs = 0;
    private static double lastDtSeconds = DT;
    /** A gap longer than this is the test doing something else, not a slow tick. */
    private static final double MAX_MEASURED_DT_S = 0.1;

    private SimHarness() {}

    private static synchronized void initHal() {
        if (!halInitialised) {
            if (!HAL.initialize(500, 0)) {
                throw new IllegalStateException("HAL failed to initialise");
            }
            halInitialised = true;
        }
    }

    /** Put the simulated driver station into enabled teleop. */
    public static void enable() {
        initHal();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    public static void disable() {
        initHal();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    /** Add a physics step, run several times per tick before the scheduler, in registration order. */
    public static void register(Runnable simStep) {
        steps.add(simStep);
    }

    private static void markTick() {
        long now = System.nanoTime();
        double dt = lastTickNs == 0 ? DT : (now - lastTickNs) / 1e9;
        lastDtSeconds = dt > MAX_MEASURED_DT_S || dt <= 0 ? DT : dt;
        lastTickNs = now;
    }

    /**
     * Wall-clock time since the previous physics step. Phoenix's simulated controller runs in real
     * time, so physics must advance by this and not by a fixed amount.
     */
    public static double lastDtSeconds() {
        return lastDtSeconds;
    }

    /** Run for {@code seconds} of wall-clock time in {@link #DT} ticks. */
    public static void run(double seconds) {
        initHal();
        int ticks = (int) Math.round(seconds / DT);
        long periodNs = Math.round(DT * 1e9);
        long deadline = System.nanoTime();
        for (int i = 0; i < ticks; i++) {
            long tickStart = deadline;
            deadline += periodNs;
            Unmanaged.feedEnable(PHOENIX_ENABLE_TIMEOUT_MS);
            DriverStationSim.notifyNewData();
            for (int k = 0; k < PHYSICS_STEPS_PER_TICK; k++) {
                long stepAt = tickStart + k * PHYSICS_STEP_NS;
                long waitNs = stepAt - System.nanoTime();
                if (waitNs > 0) {
                    sleepNanos(waitNs);
                }
                markTick();
                for (Runnable step : steps) {
                    step.run();
                }
            }
            sleepMs(PHOENIX_SETTLE_MS);
            SmartDashboard.updateValues();
            CommandScheduler.getInstance().run();
            long sleepNs = deadline - System.nanoTime();
            if (sleepNs > 0) {
                sleepNanos(sleepNs);
            }
        }
    }

    private static void sleepMs(long ms) {
        sleepNanos(ms * 1_000_000);
    }

    private static void sleepNanos(long ns) {
        try {
            Thread.sleep(ns / 1_000_000, (int) (ns % 1_000_000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted while running the simulation", e);
        }
    }

    /** Disable, drop every registered step, and cancel and unregister everything in the scheduler. */
    public static void reset() {
        disable();
        steps.clear();
        lastTickNs = 0;
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.unregisterAllSubsystems();
        scheduler.clearComposedCommands();
    }
}
