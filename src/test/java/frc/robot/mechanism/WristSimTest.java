package frc.robot.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.Setpoints;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.testutil.AlertProbe;
import frc.robot.testutil.SimHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

@Tag("sim")
class WristSimTest {
    private static final String ENCODER_ALERT = "Wrist encoder not trusted";

    private WristIOSim sim;
    private Wrist wrist;
    private final WristIOInputsAutoLogged probe = new WristIOInputsAutoLogged();
    private double maxStatorAmps;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        sim = new WristIOSim();
        wrist = new Wrist(sim);
        SimHarness.enable();
        // The simulated controller outlives each test and still holds the last test's target. Let
        // the new wrist put it in neutral before the physics is allowed to respond to it.
        SimHarness.run(0.06);
        SimHarness.register(() -> sim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14); // let the first status signals arrive
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        wrist.close();
        sim.close();
    }

    /** Run tick by tick, tracking the largest stator current the motor reports. */
    private void runRecording(double seconds) {
        int ticks = (int) Math.round(seconds / SimHarness.DT);
        for (int i = 0; i < ticks; i++) {
            SimHarness.run(SimHarness.DT);
            sim.updateInputs(probe);
            maxStatorAmps = Math.max(maxStatorAmps, Math.abs(probe.statorAmps));
        }
    }

    @Test
    void startsStowedAndTrusted() {
        // Also proves a new wrist does not act on the request the last one left in the controller.
        assertEquals(-90, sim.trueWristDeg(), 0.1);
        assertEquals(-90, wrist.wristDeg(), 1.0);
        assertTrue(wrist.isTrusted());
        assertFalse(AlertProbe.active(ENCODER_ALERT));
    }

    @ParameterizedTest
    @EnumSource(Setpoints.Pose.class)
    void reachesEverySetpoint(Setpoints.Pose p) {
        wrist.setGoalDeg(p.wristDeg);
        SimHarness.run(1.5);
        assertEquals(p.wristDeg, wrist.wristDeg(), 1.5);
        assertTrue(wrist.atGoal());
        assertEquals(p.wristDeg, sim.trueWristDeg(), 1.5, "the real wrist, not just the sensor reading");
    }

    @Test
    void staysUnderFortyAmps() {
        wrist.setGoalDeg(75);
        runRecording(1.5);
        assertEquals(75, wrist.wristDeg(), 1.5);
        assertTrue(maxStatorAmps > 1, "the simulated motor should report some current, got " + maxStatorAmps);
        assertTrue(maxStatorAmps <= 40 + 2, "stator current peaked at " + maxStatorAmps);
    }

    @Test
    void holdsAnAlgaeAtLevel() {
        sim.setGravityTorqueLbIn(151);
        wrist.setGoalDeg(0);
        SimHarness.run(2);
        assertEquals(0, wrist.wristDeg(), 1.5);
        assertEquals(0, sim.trueWristDeg(), 1.5);
    }

    @Test
    void goalIsClampedToTheSoftRange() {
        wrist.setGoalDeg(120);
        SimHarness.run(1.5);
        assertEquals(75, wrist.wristDeg(), 1.5);
    }

    @Test
    void doesNotMoveWithoutItsEncoder() {
        sim.setEncoderConnected(false);
        wrist.setGoalDeg(0);
        SimHarness.run(1);
        assertEquals(-90, wrist.wristDeg(), 2);
        assertFalse(wrist.isTrusted());
        assertFalse(wrist.atGoal());
        assertTrue(AlertProbe.active(ENCODER_ALERT));
    }

    @Test
    void losingTheEncoderMidMoveStopsTheWrist() {
        wrist.setGoalDeg(75);
        SimHarness.run(0.25);
        sim.setEncoderConnected(false);
        SimHarness.run(0.3);
        double whenLost = sim.trueWristDeg();
        SimHarness.run(0.7);
        assertFalse(wrist.isTrusted());
        // Brake mode and gravity may let it settle, but it must not keep driving up to the goal.
        assertTrue(sim.trueWristDeg() <= whenLost + 2, "kept driving after the encoder was lost");
    }

    /** An IO that reports an angle the mechanism cannot physically reach. */
    private static final class ImpossibleAngleIO implements WristIO {
        int targetsSent = 0;

        @Override
        public void updateInputs(WristIOInputs in) {
            in.connected = true;
            in.encoderConnected = true;
            in.wristDeg = 120;
            in.absoluteDeg = 120;
        }

        @Override
        public void setTargetDeg(double deg) {
            targetsSent++;
        }
    }

    /** The motor's angle disagrees with the absolute encoder and does not take a seed. */
    private static final class DisagreeingIO implements WristIO {
        int targetsSent = 0;

        @Override
        public void updateInputs(WristIOInputs in) {
            in.connected = true;
            in.encoderConnected = true;
            in.wristDeg = 0;
            in.absoluteDeg = -90;
        }

        @Override
        public void setTargetDeg(double deg) {
            targetsSent++;
        }
    }

    @Test
    void aMotorThatDisagreesWithTheEncoderIsNotTrusted() {
        var io = new DisagreeingIO();
        var suspect = new Wrist(io); // registered with the scheduler, so the harness runs its periodic
        try {
            suspect.setGoalDeg(0);
            SimHarness.run(0.2); // past the debounce
            assertFalse(suspect.isTrusted(), "a lasting disagreement is a fault");
            int sentBeforeDistrust = io.targetsSent;
            SimHarness.run(0.2);
            assertEquals(sentBeforeDistrust, io.targetsSent, "no targets once distrusted");
            assertTrue(AlertProbe.active(ENCODER_ALERT));
        } finally {
            suspect.close();
        }
    }

    /** A motor whose position is a plain number the test sets, and which takes seeds. */
    private static final class SeedableIO implements WristIO {
        double wristDeg = 0;
        double absoluteDeg = -90;
        double velocityDegPerS = 0;
        boolean reset = false;
        int seeds = 0;
        int targetsSent = 0;

        @Override
        public void updateInputs(WristIOInputs in) {
            in.connected = true;
            in.encoderConnected = true;
            in.wristDeg = wristDeg;
            in.absoluteDeg = absoluteDeg;
            in.velocityDegPerS = velocityDegPerS;
            in.resetOccurred = reset;
            reset = false;
        }

        @Override
        public void seed(double deg) {
            seeds++;
            wristDeg = deg;
        }

        @Override
        public void setTargetDeg(double deg) {
            targetsSent++;
        }
    }

    @Test
    void theMotorIsSeededFromTheEncoderAtStartUp() {
        var io = new SeedableIO(); // a freshly powered motor reads 0 wherever the wrist is
        var w = new Wrist(io);
        try {
            w.setGoalDeg(-90);
            w.periodic();
            assertEquals(0, io.targetsSent, "no motion on the loop the motor still reads a wrong angle");
            w.periodic();
            assertEquals(1, io.seeds);
            assertEquals(-90, w.wristDeg(), 1e-9);
            assertTrue(w.isTrusted());
        } finally {
            w.close();
        }
    }

    @Test
    void aMotorRebootIsDistrustedAtOnceAndReseeded() {
        var io = new SeedableIO();
        var w = new Wrist(io);
        try {
            w.setGoalDeg(30);
            w.periodic();
            w.periodic();
            io.absoluteDeg = 30;
            io.wristDeg = 0; // rebooted: the rotor count starts over
            io.reset = true;
            int sent = io.targetsSent;
            w.periodic();
            assertFalse(w.isTrusted(), "no debounce for a reboot");
            assertEquals(sent, io.targetsSent);
            w.periodic();
            assertEquals(30, w.wristDeg(), 1e-9);
            assertTrue(w.isTrusted());
        } finally {
            w.close();
        }
    }

    @Test
    void aSmallOffsetIsTakenOutOnlyWhileTheWristIsStill() {
        var io = new SeedableIO();
        var w = new Wrist(io); // registered with the scheduler, so the harness runs its periodic
        try {
            SimHarness.run(0.1);
            assertEquals(1, io.seeds);
            io.velocityDegPerS = 100;
            io.wristDeg = -88; // 2 degrees of chain slack
            SimHarness.run(0.6);
            assertEquals(1, io.seeds, "moving: the two readings are from different instants");
            io.velocityDegPerS = 0;
            SimHarness.run(0.6);
            assertEquals(2, io.seeds);
            assertEquals(-90, w.wristDeg(), 1e-9);
            io.wristDeg = -80; // 10 degrees is a fault, not slack
            SimHarness.run(0.6);
            assertEquals(2, io.seeds);
            assertFalse(w.isTrusted());
        } finally {
            w.close();
        }
    }

    @Test
    void aFastMoveStaysTrusted() {
        // The two readings are a few milliseconds apart; at full speed that must not look like a fault.
        wrist.setGoalDeg(75);
        for (int i = 0; i < 40; i++) {
            SimHarness.run(SimHarness.DT);
            assertTrue(wrist.isTrusted(), "lost trust mid-move at " + wrist.wristDeg());
        }
    }

    @Test
    void anImpossibleAbsoluteAngleIsNotTrusted() {
        var io = new ImpossibleAngleIO();
        var suspect = new Wrist(io);
        try {
            suspect.setGoalDeg(0);
            suspect.periodic();
            suspect.periodic();
            assertFalse(suspect.isTrusted());
            assertEquals(0, io.targetsSent);
            assertTrue(AlertProbe.active(ENCODER_ALERT));
        } finally {
            suspect.close();
        }
    }
}
