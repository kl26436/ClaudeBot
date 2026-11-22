package frc.robot.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.claw.AlgaeClaw;
import frc.robot.subsystems.claw.AlgaeClawIO;
import frc.robot.subsystems.claw.AlgaeClawIOInputsAutoLogged;
import frc.robot.subsystems.claw.AlgaeClawIOSim;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.coral.CoralRollersIOInputsAutoLogged;
import frc.robot.subsystems.coral.CoralRollersIOSim;
import frc.robot.testutil.AlertProbe;
import frc.robot.testutil.SimHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class RollersSimTest {
    private static final String CORAL_ALERT = "Coral sensor offline";
    private static final String ALGAE_ALERT = "Algae sensor offline";

    private CoralRollersIOSim coralSim;
    private AlgaeClawIOSim clawSim;
    private CoralRollers coral;
    private AlgaeClaw claw;
    private final CoralRollersIOInputsAutoLogged coralProbe = new CoralRollersIOInputsAutoLogged();
    private final AlgaeClawIOInputsAutoLogged clawProbe = new AlgaeClawIOInputsAutoLogged();
    private double maxClawStatorAmps;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        coralSim = new CoralRollersIOSim();
        clawSim = new AlgaeClawIOSim();
        coral = new CoralRollers(coralSim);
        claw = new AlgaeClaw(clawSim);
        SimHarness.enable();
        // The simulated controllers outlive each test and keep the last test's request: let the new
        // subsystems replace it before the physics can respond.
        SimHarness.run(0.06);
        SimHarness.register(() -> coralSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.register(() -> clawSim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14);
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        coral.close();
        claw.close();
        coralSim.close();
        clawSim.close();
    }

    private double lastVolts() {
        coralSim.updateInputs(coralProbe);
        return coralProbe.appliedVolts;
    }

    private void seat() {
        coral.intake();
        SimHarness.run(0.1);
        coralSim.setCoralPresent(true);
        SimHarness.run(0.2);
        assertTrue(coral.hasCoral(), "precondition: the coral is seated");
    }

    /** Run tick by tick, tracking the largest stator current either claw motor reports. */
    private void runRecording(double seconds) {
        int ticks = (int) Math.round(seconds / SimHarness.DT);
        for (int i = 0; i < ticks; i++) {
            SimHarness.run(SimHarness.DT);
            clawSim.updateInputs(clawProbe);
            maxClawStatorAmps = Math.max(maxClawStatorAmps, Math.abs(clawProbe.statorAmps));
        }
    }

    @Test
    void startsEmptyWithNoAlerts() {
        assertFalse(coral.hasCoral());
        assertFalse(claw.hasAlgae());
        assertFalse(AlertProbe.active(CORAL_ALERT));
        assertFalse(AlertProbe.active(ALGAE_ALERT));
    }

    @Test
    void intakeRunsInwardUntilTheCoralIsSeen() {
        coral.intake();
        SimHarness.run(0.2);
        assertEquals(4, lastVolts(), 0.2);
        assertFalse(coral.hasCoral());
        coralSim.updateInputs(coralProbe);
        assertTrue(coralProbe.velocityRps > 1, "positive volts should spin the roller inward");
    }

    @Test
    void intakeStopsWhenCoralIsSeen() {
        coral.intake();
        SimHarness.run(0.2);
        coralSim.setCoralPresent(true);
        SimHarness.run(0.2);
        assertTrue(coral.hasCoral());
        assertEquals(0.5, lastVolts(), 0.2);
    }

    @Test
    void aBounceDoesNotClearTheCoral() {
        seat();
        coralSim.setCoralPresent(false);
        SimHarness.run(0.03);
        coralSim.setCoralPresent(true);
        SimHarness.run(0.1);
        assertTrue(coral.hasCoral());
    }

    @Test
    void aCoralThatLeavesIsCleared() {
        seat();
        coral.eject();
        SimHarness.run(0.04);
        assertTrue(coral.isEjecting());
        assertTrue(lastVolts() < 0, "eject drives outward");
        coralSim.setCoralPresent(false); // the rollers push it out
        SimHarness.run(0.04);
        assertTrue(coral.hasCoral(), "a 40 ms gap is still inside the debounce");
        SimHarness.run(0.5);
        assertFalse(coral.hasCoral());
        assertTrue(coral.isEjecting(), "eject keeps running until another mode is chosen");
        // Once the roller is free the current limit no longer holds the voltage down.
        assertEquals(-6, lastVolts(), 0.3);
    }

    @Test
    void offlineSensorReadsEmptyAndAlerts() {
        seat();
        coralSim.setRangeConnected(false);
        SimHarness.run(0.2);
        assertFalse(coral.hasCoral());
        assertTrue(AlertProbe.active(CORAL_ALERT));
    }

    @Test
    void offlineAlgaeSensorReadsEmptyAndAlerts() {
        claw.intake();
        SimHarness.run(0.1);
        clawSim.setAlgaePresent(true);
        SimHarness.run(0.2);
        assertTrue(claw.hasAlgae(), "precondition: the algae is held");
        clawSim.setRangeConnected(false);
        SimHarness.run(0.2);
        assertFalse(claw.hasAlgae());
        assertTrue(AlertProbe.active(ALGAE_ALERT));
    }

    @Test
    void clawIntakeRunsInward() {
        claw.intake();
        SimHarness.run(0.6); // the 30 A limit holds the voltage down while the rollers spin up
        clawSim.updateInputs(clawProbe);
        assertEquals(6, clawProbe.appliedVolts, 0.3);
        assertTrue(clawProbe.velocityRps > 1, "positive volts should spin the rollers inward");
    }

    /** Claw IO that reports a fixed range and records what it is told. */
    private static final class RecordingClawIO implements AlgaeClawIO {
        double rangeM = 0.45;
        double lastVolts = Double.NaN;

        @Override
        public void updateInputs(AlgaeClawIOInputs in) {
            in.connected = true;
            in.rangeConnected = true;
            in.rangeM = rangeM;
        }

        @Override
        public void setVolts(double volts) {
            lastVolts = volts;
        }
    }

    @Test
    void clawIntakeSwitchesToHoldOnAlgae() {
        // Stalled on an algae, intake and hold both sit at the 30 A limit, so the applied voltage
        // cannot tell them apart. Check the command instead.
        var io = new RecordingClawIO();
        var recorded = new AlgaeClaw(io);
        try {
            recorded.intake();
            SimHarness.run(0.1);
            assertEquals(6, io.lastVolts, 1e-9);
            io.rangeM = 0.03;
            SimHarness.run(0.2);
            assertTrue(recorded.hasAlgae());
            assertEquals(1.5, io.lastVolts, 1e-9);
        } finally {
            recorded.close();
        }
    }

    @Test
    void clawStaysUnderThirtyAmps() {
        claw.intake();
        runRecording(1);
        assertTrue(maxClawStatorAmps > 1, "the simulated motors should report some current, got " + maxClawStatorAmps);
        assertTrue(maxClawStatorAmps <= 30 + 2, "stator current peaked at " + maxClawStatorAmps);
    }

    @Test
    void clawStalledOnAnAlgaeStaysUnderThirtyAmps() {
        // Holding an algae stalls both rollers; the stator limit is what keeps them cool.
        clawSim.setAlgaePresent(true);
        claw.intake();
        runRecording(1);
        assertTrue(claw.hasAlgae());
        assertTrue(maxClawStatorAmps <= 30 + 2, "stator current peaked at " + maxClawStatorAmps);
    }
}
