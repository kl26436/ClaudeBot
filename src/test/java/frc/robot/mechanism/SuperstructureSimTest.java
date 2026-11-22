package frc.robot.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import frc.robot.config.KeepOut;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.RobotConstants.WristK;
import frc.robot.config.Setpoints;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.subsystems.claw.AlgaeClaw;
import frc.robot.subsystems.claw.AlgaeClawIOSim;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.coral.CoralRollersIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.testutil.SimHarness;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class SuperstructureSimTest {
    /** How long a pose must hold before a move counts as finished. */
    private static final double SETTLE_S = 0.1;

    private ElevatorIOSim elevatorSim;
    private WristIOSim wristSim;
    private CoralRollersIOSim coralSim;
    private AlgaeClawIOSim clawSim;
    private Elevator elevator;
    private Wrist wrist;
    private CoralRollers coral;
    private AlgaeClaw claw;
    private Superstructure s;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        elevatorSim = new ElevatorIOSim();
        wristSim = new WristIOSim();
        coralSim = new CoralRollersIOSim();
        clawSim = new AlgaeClawIOSim();
        elevator = new Elevator(elevatorSim);
        wrist = new Wrist(wristSim);
        coral = new CoralRollers(coralSim);
        claw = new AlgaeClaw(clawSim);
        s = new Superstructure(elevator, wrist, coral, claw);
    }

    /** Enable and start the physics, the way every sim test fixture does. */
    private void start() {
        SimHarness.enable();
        SimHarness.run(0.06);
        SimHarness.register(() -> elevatorSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.register(() -> wristSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.register(() -> coralSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.register(() -> clawSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14);
        assertTrue(elevator.isHomed(), "precondition: the elevator homes at once from the bottom");
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        s.close();
        elevator.close();
        wrist.close();
        coral.close();
        claw.close();
        elevatorSim.close();
        wristSim.close();
        coralSim.close();
        clawSim.close();
    }

    /** How far past a soft limit each axis may coast. Beyond this the overshoot itself is a failure. */
    private static final double SOFT_LIMIT_OVERSHOOT_DEG = Setpoints.WRIST_TOL_DEG;

    private static final double SOFT_LIMIT_OVERSHOOT_IN = Setpoints.CARRIAGE_TOL_IN;

    /**
     * Whether a carriage height and wrist angle are outside every keep-out. The table's outer edges
     * (carriage 0 and 65.25 in, wrist -90 and +75) are soft travel limits, not obstacles, so a reading
     * a little past one (inside the hard stops) is judged at the limit, as the planner judges it. How
     * far past is checked separately in {@link #assertLegal}.
     */
    private static boolean legal(double carriageIn, double wristDeg) {
        return KeepOut.isLegal(
                MathUtil.clamp(carriageIn, 0, ElevatorK.SOFT_MAX_IN),
                MathUtil.clamp(wristDeg, WristK.MIN_DEG, WristK.MAX_DEG));
    }

    private void assertLegal(String what) {
        double h = elevatorSim.trueCarriageIn();
        double w = wristSim.trueWristDeg();
        String where = " at h=" + h + " w=" + w + " (reading h=" + elevator.carriageIn() + " w=" + wrist.wristDeg() + ")";
        assertTrue(legal(h, w), what + ": the mechanism is in a keep-out" + where);
        assertTrue(legal(elevator.carriageIn(), wrist.wristDeg()), what + ": the readings are in a keep-out" + where);
        assertTrue(w >= WristK.MIN_DEG - SOFT_LIMIT_OVERSHOOT_DEG && w <= WristK.MAX_DEG + SOFT_LIMIT_OVERSHOOT_DEG,
                what + ": the wrist coasted too far past a soft limit" + where);
        assertTrue(h >= -SOFT_LIMIT_OVERSHOOT_IN && h <= ElevatorK.SOFT_MAX_IN + SOFT_LIMIT_OVERSHOOT_IN,
                what + ": the carriage coasted too far past a soft limit" + where);
    }

    /**
     * Run tick by tick for at most {@code maxSeconds}, running the check after every tick. Stops early
     * once {@link Superstructure#atPose()} has held for {@link #SETTLE_S}.
     */
    private void runChecking(double maxSeconds, Runnable check) {
        int ticks = (int) Math.round(maxSeconds / SimHarness.DT);
        int settleTicks = (int) Math.round(SETTLE_S / SimHarness.DT);
        int atPoseFor = 0;
        for (int i = 0; i < ticks && atPoseFor < settleTicks; i++) {
            SimHarness.run(SimHarness.DT);
            check.run();
            atPoseFor = s.atPose() ? atPoseFor + 1 : 0;
        }
    }

    private void goTo(Wanted w) {
        s.setWanted(w);
        runChecking(3.0, () -> {});
        assertTrue(s.atPose(), "could not reach " + w);
    }

    private void seatCoral() {
        coralSim.setCoralPresent(true);
        wristSim.setGravityTorqueLbIn(WristK.GRAVITY_LBIN_CORAL);
        SimHarness.run(0.2);
        assertTrue(s.hasCoral(), "precondition: a coral is seated");
    }

    private void giveAlgae() {
        clawSim.setAlgaePresent(true);
        wristSim.setGravityTorqueLbIn(WristK.GRAVITY_LBIN_ALGAE);
        SimHarness.run(0.2);
        assertTrue(s.hasAlgae(), "precondition: an algae is held");
    }

    /**
     * Every ordered pair of different wanted states, as one walk that starts at STOW (an Eulerian
     * circuit of the complete directed graph). Each move starts where the last one ended, so every
     * transition is covered with no repositioning moves in between.
     */
    private static List<Wanted[]> everyTransition() {
        Wanted[] all = Wanted.values();
        List<Deque<Integer>> unused = new ArrayList<>();
        for (int u = 0; u < all.length; u++) {
            Deque<Integer> out = new ArrayDeque<>();
            for (int v = 0; v < all.length; v++) {
                if (v != u) {
                    out.add(v);
                }
            }
            unused.add(out);
        }
        Deque<Integer> stack = new ArrayDeque<>();
        List<Integer> circuit = new ArrayList<>();
        stack.push(Wanted.STOW.ordinal());
        while (!stack.isEmpty()) {
            int u = stack.peek();
            if (unused.get(u).isEmpty()) {
                circuit.add(stack.pop());
            } else {
                stack.push(unused.get(u).poll());
            }
        }
        Collections.reverse(circuit);
        List<Wanted[]> moves = new ArrayList<>();
        for (int i = 0; i + 1 < circuit.size(); i++) {
            moves.add(new Wanted[] {all[circuit.get(i)], all[circuit.get(i + 1)]});
        }
        return moves;
    }

    @Test
    void theTransitionWalkCoversEveryPair() {
        var moves = everyTransition();
        int n = Wanted.values().length;
        assertEquals(n * (n - 1), moves.size());
        assertEquals(n * (n - 1), moves.stream().map(m -> m[0] + ">" + m[1]).distinct().count());
        for (int i = 0; i + 1 < moves.size(); i++) {
            assertEquals(moves.get(i)[1], moves.get(i + 1)[0], "each move starts where the last ended");
        }
    }

    @Test
    void everyTransitionStaysLegalInPhysics() {
        start();
        for (Wanted[] move : everyTransition()) {
            Wanted from = move[0];
            Wanted to = move[1];
            assertTrue(s.atPose(), "should already be at " + from);
            s.setWanted(to);
            runChecking(3.0, () -> assertLegal(from + " to " + to));
            assertTrue(s.atPose(), from + " to " + to + " did not finish");
        }
    }

    @Test
    void atPoseMatchesTheSetpoint() {
        start();
        goTo(Wanted.L3);
        assertEquals(Setpoints.Pose.L3.carriageIn, elevatorSim.trueCarriageIn(), Setpoints.CARRIAGE_TOL_IN + 0.1);
        assertEquals(Setpoints.Pose.L3.wristDeg, wristSim.trueWristDeg(), Setpoints.WRIST_TOL_DEG + 0.5);
    }

    @Test
    void releaseWaitsForThePose() {
        start();
        seatCoral();
        s.setWanted(Wanted.L4);
        s.requestRelease(true);
        SimHarness.run(0.2);
        assertFalse(coral.isEjecting());
        SimHarness.run(2.5);
        assertTrue(s.atPose());
        assertTrue(coral.isEjecting());
    }

    @Test
    void releaseEjectsOnlyTheMatchingPiece() {
        start();
        seatCoral();
        giveAlgae();
        goTo(Wanted.NET);
        s.requestRelease(true);
        SimHarness.run(0.1);
        assertTrue(claw.isEjecting(), "the net takes the algae");
        assertFalse(coral.isEjecting(), "the coral stays put at the net");
    }

    @Test
    void aNewWantedClearsThePendingRelease() {
        start();
        seatCoral();
        s.setWanted(Wanted.L2);
        s.requestRelease(true);
        SimHarness.run(0.1);
        s.setWanted(Wanted.L3);
        SimHarness.run(2.0);
        assertTrue(s.atPose());
        assertFalse(coral.isEjecting(), "the release was for L2, not L3");
    }

    @Test
    void stationIntakeRunsTheCoralRollersAndAlgaePosesRunTheClaw() {
        start();
        goTo(Wanted.STATION_INTAKE);
        coralSim.setCoralPresent(true);
        SimHarness.run(0.2);
        assertTrue(s.hasCoral(), "the rollers took the coral in and hold it");
        goTo(Wanted.ALGAE_LOW);
        clawSim.setAlgaePresent(true);
        SimHarness.run(0.2);
        assertTrue(s.hasAlgae());
    }

    @Test
    void stowWithAlgaeUsesCarry() {
        start();
        giveAlgae();
        s.setWanted(Wanted.STOW);
        SimHarness.run(2.5);
        assertTrue(wrist.wristDeg() >= 10);
        assertTrue(elevator.carriageIn() >= 1.25);
        assertTrue(wristSim.trueWristDeg() >= 10);
        assertTrue(s.atPose(), "carry is the stow pose while holding an algae");
    }

    @Test
    void cancelMidScoreReturnsLegallyAndKeepsTheSpeedCap() {
        start();
        seatCoral();
        s.setWanted(Wanted.L4);
        SimHarness.run(0.6);
        s.setWanted(Wanted.STOW);
        runChecking(3.0, () -> {
            assertLegal("L4 cancelled to STOW");
            if (elevator.carriageIn() > 23.5) {
                assertTrue(s.driveSpeedScale() < 1.0, "speed cap lost at " + elevator.carriageIn() + " in");
            }
        });
        assertTrue(s.atPose(), "did not reach stow: h=" + elevator.carriageIn() + " w=" + wrist.wristDeg());
    }

    /**
     * The driver lets go of a score with the carriage high and falling, then presses again while it is
     * still sliding down: the carriage cannot stop at once, so the wrist must not swing back to L4's
     * -52 until the carriage is sure to stay above the funnel band.
     */
    @Test
    void cancelAndPressAgainWhileFallingStaysLegal() {
        start();
        seatCoral();
        for (double fallS : new double[] {0.0, 0.2, 0.4}) {
            goTo(Wanted.L4);
            s.setWanted(Wanted.STOW);
            // The wrist turns first; wait for the carriage to be falling fast, then a little longer.
            for (int i = 0; i < 75 && elevator.velocityInPerS() > -40; i++) {
                SimHarness.run(SimHarness.DT);
                assertLegal("L4 cancelled");
            }
            runChecking(fallS, () -> assertLegal("L4 cancelled"));
            assertTrue(elevator.velocityInPerS() < -20, "precondition: falling, at " + elevator.velocityInPerS() + " in/s");
            assertTrue(elevator.carriageIn() > 23, "precondition: still above the funnel band, at " + elevator.carriageIn());
            s.setWanted(Wanted.L4);
            runChecking(3.0, () -> assertLegal("pressed again after " + fallS + " s of falling"));
            assertTrue(s.atPose(), "did not get back to L4 after " + fallS + " s");
        }
    }

    @Test
    void driveSpeedScaleFollowsTheCarriage() {
        start();
        assertEquals(1.0, s.driveSpeedScale(), 1e-9);
        goTo(Wanted.L4);
        assertEquals(0.35, s.driveSpeedScale(), 0.02);
        goTo(Wanted.L3); // 33.3 in: (33.3 - 23) / (65.25 - 23) of the way to 0.35
        assertEquals(1.0 - 0.65 * (33.3 - 23) / (65.25 - 23), s.driveSpeedScale(), 0.02);
    }

    @Test
    void untrustedWristKeepsTheElevatorDown() {
        start();
        wristSim.setEncoderConnected(false);
        s.setWanted(Wanted.L3);
        SimHarness.run(2);
        assertTrue(elevator.carriageIn() <= 1.25);
        assertTrue(elevatorSim.trueCarriageIn() <= 1.25);
        assertFalse(s.atPose());
    }

    @Test
    void wristLostWithTheElevatorUpHoldsTheCarriageStill() {
        start();
        goTo(Wanted.L3);
        wristSim.setEncoderConnected(false);
        SimHarness.run(0.1);
        double whenLost = elevatorSim.trueCarriageIn();
        s.setWanted(Wanted.STOW);
        SimHarness.run(1.5);
        // With the wrist angle unknown, driving down could put it through the funnel: stay put.
        assertEquals(whenLost, elevatorSim.trueCarriageIn(), 1.0);
    }

    @Test
    void unhomedElevatorKeepsTheWristStill() {
        // Start part-way up with a dead home switch: homing takes over three seconds.
        elevatorSim.setHomeSwitchBroken(true);
        elevatorSim.setCarriageIn(5);
        SimHarness.enable();
        SimHarness.run(0.06);
        SimHarness.register(() -> elevatorSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.register(() -> wristSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14);
        s.setWanted(Wanted.L3);
        SimHarness.run(1.5);
        assertFalse(elevator.isHomed());
        assertEquals(-90, wristSim.trueWristDeg(), 2);
        assertFalse(s.atPose());
    }
}
