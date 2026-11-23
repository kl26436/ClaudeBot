package frc.robot.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.RobotConstants.ClimberK;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.State;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.testutil.AlertProbe;
import frc.robot.testutil.SimHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** The climb sequence of drawings CL-000 and R-001 on simulated winch, pawls and latch. */
@Tag("sim")
class ClimberSimTest {
    private ClimberIOSim sim;
    private Climber climber;
    private boolean poseReady;
    private double minForkDeg;
    private double maxForkDeg;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        sim = new ClimberIOSim();
        climber = new Climber(sim, () -> poseReady, () -> poseReady);
        poseReady = true;
        SimHarness.enable();
        SimHarness.run(0.06);
        SimHarness.register(() -> sim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14);
        minForkDeg = 1e9;
        maxForkDeg = -1e9;
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        climber.close();
        sim.close();
    }

    /** Run up to {@code seconds} with the button as given, until the state is reached. */
    private void runUntil(State wanted, boolean held, double seconds) {
        for (int i = 0; i < seconds / SimHarness.DT && climber.state() != wanted; i++) {
            step(held);
        }
        assertEquals(wanted, climber.state(), "fork at " + sim.trueForkDeg());
    }

    private void step(boolean held) {
        climber.setHeld(held);
        SimHarness.run(SimHarness.DT);
        minForkDeg = Math.min(minForkDeg, sim.trueForkDeg());
        maxForkDeg = Math.max(maxForkDeg, sim.trueForkDeg());
    }

    private void hold(boolean held, double seconds) {
        for (int i = 0; i < seconds / SimHarness.DT; i++) {
            step(held);
        }
    }

    private void deploy() {
        runUntil(State.DEPLOYED, true, 8);
    }

    @Test
    void startsStowedAndStaysThere() {
        hold(false, 0.5);
        assertEquals(State.STOWED, climber.state());
        assertEquals(90, sim.trueForkDeg(), 0.5);
        assertFalse(climber.committed());
        assertEquals(0, sim.trayDeg());
        assertFalse(sim.pawlsUp());
    }

    @Test
    void nothingFiresUntilTheMechanismIsInTheClimbPose() {
        poseReady = false;
        hold(true, 1.5);
        assertEquals(State.PREPARING, climber.state());
        assertFalse(climber.committed());
        assertEquals(0, sim.trayDeg(), "the tray must not open into the wrist");
        assertEquals(90, sim.trueForkDeg(), 0.5);
        hold(false, 0.1);
        assertEquals(State.STOWED, climber.state(), "letting go before the latch fires undoes nothing");
    }

    @Test
    void deploysInTheDrawingsOrder() {
        runUntil(State.OPENING, true, 1);
        assertTrue(climber.committed());
        assertEquals(90, sim.trueForkDeg(), 0.5, "no fork motion until the tray is open");
        runUntil(State.TENSIONING, true, 2);
        assertEquals(90, sim.trayDeg(), 0.01);
        assertFalse(sim.pawlsUp());
        runUntil(State.LIFTING, true, 1);
        assertTrue(sim.trueForkDeg() > 90 + 1, "wound in to take the load off the pawls");
        runUntil(State.PAYING_OUT, true, 1);
        assertTrue(sim.pawlsUp(), "paying out with a pawl down would strip it");
        runUntil(State.DEPLOYED, true, 4);
        assertEquals(0, sim.trueForkDeg(), 2.5);
        assertTrue(maxForkDeg < 97, "kept off the hard stop");
        hold(false, 0.5);
        assertFalse(sim.pawlsUp(), "pawls down before the robot's weight goes on");
        assertEquals(0, sim.trueForkDeg(), 4.0);
    }

    @Test
    void holdingTheButtonThroughDoesNotStartTheClimb() {
        deploy();
        hold(true, 1.0);
        assertEquals(State.DEPLOYED, climber.state(), "the driver has to back onto the cage and press again");
        assertEquals(0, sim.trueForkDeg(), 4.0);
    }

    @Test
    void climbsToEightyEightAndTheRatchetsHold() {
        deploy();
        hold(false, 0.3);
        sim.setCageCaptured(true);
        runUntil(State.CLIMBED, true, 6);
        assertEquals(ClimberK.CLIMBED_DEG, sim.trueForkDeg(), 2.0);
        hold(false, 0.5);
        SimHarness.disable();
        double settled = sim.trueForkDeg();
        for (int i = 0; i < 50; i++) {
            SimHarness.run(SimHarness.DT);
        }
        assertTrue(sim.trueForkDeg() > ClimberK.CLIMBED_DEG - 2 - ClimberK.TOOTH_DEG, "held at " + sim.trueForkDeg());
        assertEquals(settled, sim.trueForkDeg(), ClimberK.TOOTH_DEG);
        assertTrue(maxForkDeg < 96, "must stop short of the 97 degree stop, reached " + maxForkDeg);
    }

    @Test
    void lettingGoWhilePayingOutStopsTheForkAndItResumes() {
        runUntil(State.PAYING_OUT, true, 5);
        hold(true, 0.6);
        hold(false, 0.6);
        double paused = sim.trueForkDeg();
        assertTrue(paused > 5 && paused < 85, "part way: " + paused);
        assertFalse(sim.pawlsUp());
        hold(false, 0.5);
        assertEquals(paused, sim.trueForkDeg(), ClimberK.TOOTH_DEG, "held while the button is up");
        runUntil(State.DEPLOYED, true, 5);
        assertEquals(0, sim.trueForkDeg(), 4.0);
    }

    @Test
    void lettingGoWhileWindingHoldsOnTheRatchet() {
        deploy();
        hold(false, 0.3);
        sim.setCageCaptured(true);
        runUntil(State.WINDING, true, 1);
        hold(true, 0.8);
        hold(false, 0.6);
        double paused = sim.trueForkDeg();
        assertTrue(paused > 5 && paused < 85, "part way: " + paused);
        hold(false, 0.6);
        assertEquals(paused, sim.trueForkDeg(), ClimberK.TOOTH_DEG);
        runUntil(State.CLIMBED, true, 6);
    }

    @Test
    void theForkStopsPayingOutIfTheWristLeavesTheClimbPose() {
        runUntil(State.PAYING_OUT, true, 5);
        hold(true, 0.6);
        poseReady = false;
        hold(true, 0.4);
        double stopped = sim.trueForkDeg();
        hold(true, 0.6);
        assertEquals(stopped, sim.trueForkDeg(), ClimberK.TOOTH_DEG);
    }

    @Test
    void aWristFaultDoesNotStopAClimbUnderLoad() {
        deploy();
        hold(false, 0.3);
        sim.setCageCaptured(true);
        runUntil(State.WINDING, true, 1);
        hold(true, 0.5);
        poseReady = false;
        runUntil(State.CLIMBED, true, 6);
    }

    @Test
    void disablingMidDeployDropsThePawls() {
        runUntil(State.PAYING_OUT, true, 5);
        hold(true, 0.6);
        climber.setHeld(true);
        SimHarness.disable();
        SimHarness.run(0.6);
        assertFalse(sim.pawlsUp());
        double stopped = sim.trueForkDeg();
        SimHarness.run(0.5);
        assertEquals(stopped, sim.trueForkDeg(), ClimberK.TOOTH_DEG);
    }

    /** A winch whose reading the test sets. */
    private static final class ScriptedIO implements ClimberIO {
        double forkDeg = 90;
        double velocityDegPerS = 0;
        double statorAmps = 0;
        boolean reset = false;
        int targets = 0;
        boolean lifted = false;
        boolean open = false;

        @Override
        public void updateInputs(ClimberIOInputs in) {
            in.connected = true;
            in.followerConnected = true;
            in.forkDeg = forkDeg;
            in.velocityDegPerS = velocityDegPerS;
            in.statorAmps = statorAmps;
            in.resetOccurred = reset;
            reset = false;
        }

        @Override
        public void setTargetDeg(double deg) {
            targets++;
        }

        @Override
        public void setPawlsLifted(boolean l) {
            lifted = l;
        }

        @Override
        public void setFunnelOpen(boolean o) {
            open = o;
        }
    }

    private void withScripted(java.util.function.BiConsumer<ScriptedIO, Climber> body, double startDeg) {
        var io = new ScriptedIO();
        io.forkDeg = startDeg;
        var c = new Climber(io, () -> true, () -> true);
        try {
            body.accept(io, c);
        } finally {
            c.close();
        }
    }

    @Test
    void aCodeRestartWithTheForkOutComesBackDeployed() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            assertEquals(State.DEPLOYED, c.state());
            assertTrue(c.committed(), "the mechanism must not leave the climb pose");
            assertTrue(io.open);
            assertFalse(io.lifted);
            c.setHeld(true);
            SimHarness.run(0.1);
            assertEquals(State.WINDING, c.state());
        }, 10);
    }

    @Test
    void aMotorRebootWithTheForkOutStopsTheWinch() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            c.setHeld(true);
            SimHarness.run(0.1);
            io.forkDeg = 90;
            io.reset = true;
            SimHarness.run(0.04);
            int sent = io.targets;
            SimHarness.run(0.2);
            assertEquals(State.LOST, c.state());
            assertEquals(sent, io.targets);
            assertFalse(io.lifted);
            assertTrue(AlertProbe.active("Climber lost its position: winch stopped"));
        }, 10);
    }

    @Test
    void aStalledWinchStopsUntilTheButtonIsPressedAgain() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            io.statorAmps = 60;
            io.velocityDegPerS = 0;
            c.setHeld(true);
            SimHarness.run(1.5);
            int sent = io.targets;
            assertTrue(sent > 0);
            SimHarness.run(0.4);
            assertEquals(sent, io.targets, "stopped pushing against whatever it hit");
            assertTrue(AlertProbe.active("Climber winch stalled"));
            io.statorAmps = 0;
            c.setHeld(false);
            SimHarness.run(0.1);
            c.setHeld(true);
            SimHarness.run(0.1);
            assertTrue(io.targets > sent);
        }, 10);
    }
}
