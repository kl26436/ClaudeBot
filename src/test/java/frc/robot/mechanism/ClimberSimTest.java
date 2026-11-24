package frc.robot.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.RobotConstants.ClimberK;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.State;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.CommitMemory;
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
    private boolean wristTrusted;
    private CommitMemory memory;
    private double minForkDeg;
    private double maxForkDeg;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        sim = new ClimberIOSim();
        memory = CommitMemory.inMemory("");
        climber = new Climber(sim, new Climber.Mechanism(() -> poseReady, () -> poseReady, () -> !poseReady && wristTrusted), memory);
        wristTrusted = true;
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
        assertEquals("", memory.load());
        assertEquals(0, sim.trayDeg(), "the tray must not open into the wrist");
        assertEquals(90, sim.trueForkDeg(), 0.5);
        hold(false, 0.1);
        assertEquals(State.STOWED, climber.state(), "letting go before the latch fires undoes nothing");
    }

    /** Wherever the pawls rest between teeth when the wind-in starts, the fork gets out. */
    @org.junit.jupiter.params.ParameterizedTest
    @org.junit.jupiter.params.provider.ValueSource(doubles = {89.0, 90.9, 91.7, 91.9, 92.6})
    void deploysFromAnywhereBetweenTwoTeeth(double startDeg) {
        runUntil(State.OPENING, true, 1.5);
        sim.setRopeDeg(startDeg);
        runUntil(State.DEPLOYED, true, 9);
        assertEquals(0, sim.trueForkDeg(), 2.5);
        assertTrue(maxForkDeg < 96);
    }

    @Test
    void aShortPressFiresNothing() {
        hold(true, 0.5);
        hold(false, 0.3);
        assertEquals(State.STOWED, climber.state());
        assertEquals(0, sim.trayDeg());
    }

    @Test
    void deploysInTheDrawingsOrder() {
        runUntil(State.OPENING, true, 1);
        assertEquals(CommitMemory.PRACTICE, memory.load(), "remembered before anything moves");
        assertTrue(climber.committed());
        assertEquals(90, sim.trueForkDeg(), 0.5, "no fork motion until the tray is open");
        runUntil(State.TENSIONING, true, 2);
        assertEquals(90, sim.trayDeg(), 0.01);
        assertFalse(sim.pawlsUp());
        runUntil(State.LIFTING, true, 1);
        assertTrue(sim.trueForkDeg() > 90 + 0.8, "wound in to take the load off the pawls");
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

    @Test
    void aBumpOfTheButtonWithTheForkOutDoesNotWind() {
        deploy();
        hold(false, 0.2);
        hold(true, 0.2);
        hold(false, 0.2);
        assertEquals(State.DEPLOYED, climber.state());
    }

    private void holdLower(boolean held, double seconds) {
        climber.setHeld(false);
        for (int i = 0; i < seconds / SimHarness.DT; i++) {
            climber.setLowerHeld(held);
            SimHarness.run(SimHarness.DT);
        }
    }

    @Test
    void anEmptyForkCanBeLetBackDown() {
        deploy();
        hold(false, 0.3);
        runUntil(State.CLIMBED, true, 6); // missed the cage
        hold(false, 0.3);
        holdLower(true, 5);
        assertEquals(State.DEPLOYED, climber.state());
        assertEquals(0, sim.trueForkDeg(), 4);
        holdLower(false, 0.3);
        assertFalse(sim.pawlsUp());
        sim.setCageCaptured(true);
        runUntil(State.CLIMBED, true, 6);
    }

    @Test
    void aHangingRobotIsNotLowered() {
        deploy();
        hold(false, 0.3);
        sim.setCageCaptured(true);
        runUntil(State.CLIMBED, true, 6);
        hold(false, 0.5);
        double before = sim.trueForkDeg();
        holdLower(true, 3);
        assertTrue(sim.trueForkDeg() >= before - 0.5, "lowered from " + before + " to " + sim.trueForkDeg());
        assertFalse(sim.pawlsUp());
        assertTrue(AlertProbe.active("Fork is loaded: it will not be lowered"));
        holdLower(false, 0.5);
        assertTrue(sim.trueForkDeg() >= before - ClimberK.TOOTH_DEG);
    }

    @Test
    void aPullDoesNotStartWithTheWristInTheWay() {
        deploy();
        hold(false, 0.3);
        sim.setCageCaptured(true);
        runUntil(State.WINDING, true, 1);
        hold(true, 0.6);
        hold(false, 0.6);
        double paused = sim.trueForkDeg();
        poseReady = false; // it drooped while the robot was disabled
        hold(true, 1.0);
        assertEquals(paused, sim.trueForkDeg(), ClimberK.TOOTH_DEG);
        poseReady = true;
        runUntil(State.CLIMBED, true, 6);
    }

    @Test
    void aFaultyWristSensorDoesNotStrandARobotPartWayUp() {
        deploy();
        hold(false, 0.3);
        sim.setCageCaptured(true);
        runUntil(State.WINDING, true, 1);
        hold(true, 0.6);
        hold(false, 0.6);
        poseReady = false;
        wristTrusted = false;
        runUntil(State.CLIMBED, true, 6);
    }

    @Test
    void theTrayGetsItsFullTimeAfterADisable() {
        runUntil(State.OPENING, true, 1);
        climber.setHeld(true);
        SimHarness.disable();
        SimHarness.run(1.5);
        SimHarness.enable();
        hold(true, 0.5);
        assertEquals(State.OPENING, climber.state());
        runUntil(State.TENSIONING, true, 1);
    }

    /** A winch whose reading the test sets. */
    private static final class ScriptedIO implements ClimberIO {
        double forkDeg = 90;
        double velocityDegPerS = 0;
        double statorAmps = 0;
        boolean reset = false;
        boolean connected = true;
        int targets = 0;
        double target = Double.NaN;
        boolean lifted = false;
        boolean open = false;
        int zeroed = 0;

        @Override
        public void updateInputs(ClimberIOInputs in) {
            in.connected = connected;
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
            target = deg;
        }

        @Override
        public void zeroStowed() {
            zeroed++;
            forkDeg = 90;
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

    private void withScripted(java.util.function.BiConsumer<ScriptedIO, Climber> body, double startDeg, CommitMemory memory) {
        var io = new ScriptedIO();
        io.forkDeg = startDeg;
        var c = new Climber(io, new Climber.Mechanism(() -> true, () -> true, () -> false), memory);
        try {
            body.accept(io, c);
        } finally {
            c.close();
        }
    }

    private static CommitMemory begun() {
        return CommitMemory.inMemory(CommitMemory.PRACTICE);
    }

    private static final String LOST = "Climber does not know where its fork is: winch stopped, needs the pit reset";

    private static void press(Climber c, double seconds) {
        c.setHeld(false);
        SimHarness.run(0.1);
        c.setHeld(true);
        SimHarness.run(seconds);
    }

    @Test
    void aCodeRestartWithTheForkOutComesBackDeployed() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            assertEquals(State.DEPLOYED, c.state());
            assertTrue(c.committed(), "the mechanism must not leave the climb pose");
            assertFalse(io.open, "a restart does not fire the latch again");
            assertFalse(io.lifted);
            press(c, 0.6);
            assertEquals(State.WINDING, c.state());
        }, 10, begun());
    }

    @Test
    void aCodeRestartPartWayOutCanStillBePaidOut() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            assertEquals(State.DEPLOYED, c.state());
            c.setLowerHeld(true);
            SimHarness.run(0.1);
            assertEquals(State.TENSIONING, c.state());
        }, 45, begun());
    }

    /** 84 to 89 degrees is a robot hanging on its ratchets. Treated as stowed, a press would let it down. */
    @Test
    void aCodeRestartWhileHangingStaysClimbed() {
        for (double deg : new double[] {81, 84.4, 88, 89}) {
            withScripted((io, c) -> {
                c.setHeld(false);
                SimHarness.run(0.1);
                assertEquals(State.CLIMBED, c.state(), "fork " + deg);
                assertTrue(c.committed());
                press(c, 3.0);
                assertFalse(io.lifted);
                assertFalse(io.open);
                assertTrue(Double.isNaN(io.target) || io.target == ClimberK.CLIMBED_DEG, "sent to " + io.target);
            }, deg, begun());
        }
    }

    /** A motor controller that lost power mid-climb reads exactly stowed, wherever the fork is. */
    @Test
    void aPowerLossMidClimbStopsTheWinch() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            assertEquals(State.LOST, c.state());
            assertTrue(c.committed());
            press(c, 3.0);
            assertEquals(0, io.targets);
            assertFalse(io.lifted);
            assertTrue(AlertProbe.active(LOST));
        }, 90, begun());
    }

    @Test
    void aForkOutWithNoClimbRememberedStopsTheWinch() {
        withScripted((io, c) -> {
            press(c, 3.0);
            assertEquals(State.LOST, c.state());
            assertFalse(io.open);
            assertEquals(0, io.targets);
        }, 40, CommitMemory.inMemory(""));
    }

    @Test
    void thePitResetForgetsTheClimbOnlyWhileDisabled() {
        var memory = begun();
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            c.pitReset();
            assertTrue(c.committed(), "ignored while enabled");
            SimHarness.disable();
            SimHarness.run(0.1);
            c.pitReset();
            SimHarness.run(0.1);
            assertFalse(c.committed());
            assertEquals(State.STOWED, c.state());
            assertEquals(1, io.zeroed);
            assertEquals("", memory.load());
            SimHarness.enable();
            SimHarness.run(0.1);
            assertFalse(io.open, "nothing fires when it is next enabled");
        }, 86, memory);
    }

    /** Forgetting the pit reset must not cost the next match. */
    @Test
    void theNextMatchStartsCleanWithTheForkStowed() {
        for (double deg : new double[] {90, 40}) {
            var memory = new CommitMemory() {
                String saved = "TEST Qualification 4.1";

                @Override
                public String load() {
                    return saved;
                }

                @Override
                public void save(String m) {
                    saved = m;
                }

                @Override
                public String currentMatch() {
                    return "TEST Qualification 5.1";
                }
            };
            withScripted((io, c) -> {
                SimHarness.disable();
                SimHarness.run(0.2);
                assertEquals(deg < 80, c.committed(), "fork " + deg);
                SimHarness.enable();
            }, deg, memory);
        }
    }

    @Test
    void aMotorRebootWithTheForkOutStopsTheWinch() {
        withScripted((io, c) -> {
            press(c, 0.6);
            io.forkDeg = 90;
            io.reset = true;
            SimHarness.run(0.04);
            int sent = io.targets;
            SimHarness.run(0.2);
            assertEquals(State.LOST, c.state());
            assertEquals(sent, io.targets);
            assertFalse(io.lifted);
            assertTrue(AlertProbe.active(LOST));
        }, 10, begun());
    }

    @Test
    void aMotorRebootBeforeTheForkHasMovedCarriesOn() {
        withScripted((io, c) -> {
            press(c, 2.2); // past the tray opening, winding in with the fork still at stow
            assertEquals(State.TENSIONING, c.state());
            io.reset = true;
            SimHarness.run(0.04);
            assertTrue(c.state() == State.OPENING || c.state() == State.TENSIONING, "in " + c.state());
        }, 90, CommitMemory.inMemory(""));
    }

    /** Stowed against the 97 degree stop, the few degrees of wind-in run into the bolts. */
    @Test
    void windingIntoTheStopBoltsCountsAsTight() {
        withScripted((io, c) -> {
            press(c, 2.2);
            assertEquals(State.TENSIONING, c.state());
            io.statorAmps = 40;
            for (int i = 0; i < 20 && c.state() == State.TENSIONING; i++) {
                SimHarness.run(0.02);
            }
            io.statorAmps = 3; // told to stay where it is, the motor stops pushing
            SimHarness.run(0.04);
            assertEquals(State.LIFTING, c.state(), "must not push on the bolts until it stalls");
            assertEquals(90, io.target, 0.1, "holds where it stopped");
        }, 90, CommitMemory.inMemory(""));
    }

    @Test
    void aLostMotorLetsThePawlsDown() {
        withScripted((io, c) -> {
            press(c, 2.2);
            io.forkDeg = 91.8;
            for (int i = 0; i < 60 && c.state() != State.PAYING_OUT; i++) {
                SimHarness.run(0.02);
            }
            SimHarness.run(0.06);
            assertEquals(State.PAYING_OUT, c.state());
            assertTrue(io.lifted);
            io.connected = false;
            SimHarness.run(0.7);
            assertFalse(io.lifted);
            assertTrue(AlertProbe.active("Climber motor not answering"));
        }, 90, CommitMemory.inMemory(""));
    }

    @Test
    void aStalledWinchStopsUntilTheButtonIsPressedAgain() {
        withScripted((io, c) -> {
            c.setHeld(false);
            SimHarness.run(0.1);
            io.statorAmps = 60;
            io.velocityDegPerS = 0;
            c.setHeld(true);
            SimHarness.run(1.8);
            int sent = io.targets;
            assertTrue(sent > 0);
            SimHarness.run(0.4);
            assertEquals(sent, io.targets, "stopped pushing against whatever it hit");
            assertTrue(AlertProbe.active("Climber winch stalled"));
            io.statorAmps = 0;
            press(c, 0.1);
            assertTrue(io.targets > sent);
        }, 10, begun());
    }
}
