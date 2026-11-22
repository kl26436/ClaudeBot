package frc.robot.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Setpoints;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.testutil.SimHarness;
import java.util.Arrays;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

@Tag("sim")
class ElevatorSimTest {
    private static final String HOME_ALERT = "Elevator home switch not seen";
    private static final double SOFT_MAX_IN = 65.25;
    private static final double MAX_DOWN_ACCEL_IN_PER_S2 = 231.65;
    private static final int ACCEL_WINDOW_TICKS = 3;

    private ElevatorIOSim sim;
    private Elevator elevator;

    private double maxHeight;
    private double minAccelInPerS2;
    private final java.util.ArrayDeque<double[]> recentSamples = new java.util.ArrayDeque<>();

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        sim = new ElevatorIOSim();
        elevator = new Elevator(sim);
        SimHarness.register(() -> sim.simulate(SimHarness.lastDtSeconds()));
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        elevator.close();
        sim.close();
    }

    /** Enable, wait for homing to finish, then run the action. */
    private void homeThen(Runnable action) {
        SimHarness.enable();
        SimHarness.run(1.0);
        assertTrue(elevator.isHomed(), "elevator should be homed before the action");
        action.run();
    }

    /**
     * Run tick by tick, tracking the highest carriage height and the lowest acceleration seen.
     * Phoenix status signals arrive asynchronously, so a single 20 ms difference of the velocity
     * can show a spike when one sample repeats and the next jumps. Acceleration is therefore the
     * velocity change over {@link #ACCEL_WINDOW_TICKS} ticks (60 ms) using real timestamps.
     */
    private void runRecording(double seconds) {
        maxHeight = Math.max(maxHeight, elevator.carriageIn());
        int ticks = (int) Math.round(seconds / SimHarness.DT);
        for (int i = 0; i < ticks; i++) {
            SimHarness.run(SimHarness.DT);
            maxHeight = Math.max(maxHeight, elevator.carriageIn());
            recentSamples.addLast(new double[] {System.nanoTime() / 1e9, elevator.velocityInPerS()});
            if (recentSamples.size() > ACCEL_WINDOW_TICKS + 1) {
                recentSamples.removeFirst();
            }
            if (recentSamples.size() == ACCEL_WINDOW_TICKS + 1) {
                double[] first = recentSamples.peekFirst();
                double[] last = recentSamples.peekLast();
                minAccelInPerS2 = Math.min(minAccelInPerS2, (last[1] - first[1]) / (last[0] - first[0]));
            }
        }
    }

    private static boolean alertActive(String text) {
        String[] warnings = NetworkTableInstance.getDefault()
                .getStringArrayTopic("/SmartDashboard/Alerts/warnings")
                .subscribe(new String[0])
                .get();
        return Arrays.asList(warnings).contains(text);
    }

    @Test
    void homesFromAnUnknownHeight() {
        sim.setCarriageIn(7.0);
        SimHarness.enable();
        SimHarness.run(4);
        assertTrue(elevator.isHomed());
        assertEquals(0, elevator.carriageIn(), 0.3);
        assertFalse(alertActive(HOME_ALERT), "a working switch must not raise the alert");
    }

    @ParameterizedTest
    @EnumSource(Setpoints.Pose.class)
    void reachesEverySetpoint(Setpoints.Pose p) {
        homeThen(() -> elevator.setGoalIn(p.carriageIn));
        SimHarness.run(2.0);
        assertEquals(p.carriageIn, elevator.carriageIn(), 0.5);
    }

    @Test
    void neverPassesTheSoftLimit() {
        homeThen(() -> elevator.setGoalIn(80));
        runRecording(2.5);
        assertTrue(maxHeight > SOFT_MAX_IN - 0.5, "should have reached the top, got " + maxHeight);
        assertTrue(maxHeight <= SOFT_MAX_IN + 0.3, "passed the soft limit: " + maxHeight);
    }

    @Test
    void downwardAccelerationStaysUnderPointSixG() {
        homeThen(() -> elevator.setGoalIn(SOFT_MAX_IN));
        runRecording(2.0);
        assertEquals(SOFT_MAX_IN, elevator.carriageIn(), 0.5, "should be at the top before coming down");
        minAccelInPerS2 = 0;
        recentSamples.clear();
        elevator.setGoalIn(0);
        runRecording(2.0);
        assertTrue(minAccelInPerS2 < -100, "the move down should actually accelerate, got " + minAccelInPerS2);
        // 15 % allowance for differentiation noise.
        assertTrue(minAccelInPerS2 >= -MAX_DOWN_ACCEL_IN_PER_S2 * 1.15, "downward accel " + minAccelInPerS2);
    }

    @Test
    void fullTravelTakesUnderOnePointFiveSeconds() {
        homeThen(() -> elevator.setGoalIn(SOFT_MAX_IN));
        assertFalse(elevator.atGoal(), "the move has not started yet");
        int ticks = 0;
        int maxTicks = (int) Math.round(3.0 / SimHarness.DT);
        while (!elevator.atGoal() && ticks < maxTicks) {
            SimHarness.run(SimHarness.DT);
            ticks++;
        }
        double elapsed = ticks * SimHarness.DT;
        assertTrue(elevator.atGoal(), "never reached the top");
        assertTrue(elapsed <= 1.5, "took " + elapsed + " s");
        // The motion profile cannot beat cruise speed and acceleration; a faster result means the sim is wrong.
        assertTrue(elapsed >= 0.9, "implausibly fast: " + elapsed + " s");
    }

    @Test
    void brokenHomeSwitchStillHomesOnStall() {
        assertFalse(alertActive(HOME_ALERT), "alert already active before the test");
        sim.setHomeSwitchBroken(true);
        sim.setCarriageIn(5);
        SimHarness.enable();
        SimHarness.run(5);
        assertTrue(elevator.isHomed());
        assertTrue(alertActive(HOME_ALERT));
        assertEquals(0, elevator.carriageIn(), 0.3);
    }

    @Test
    void ignoresGoalsUntilHomed() {
        elevator.setGoalIn(30);
        SimHarness.run(0.2);
        assertTrue(elevator.carriageIn() < 8);
        assertFalse(elevator.isHomed());
        // The early goal was dropped, not queued: once homed the carriage stays down.
        SimHarness.enable();
        SimHarness.run(2.0);
        assertTrue(elevator.isHomed());
        assertTrue(elevator.carriageIn() < 1.0, "queued goal moved the carriage to " + elevator.carriageIn());
    }
}
