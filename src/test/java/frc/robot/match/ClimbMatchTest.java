package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimbCommands;
import frc.robot.config.KeepOut;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.subsystems.climber.Climber.State;
import frc.robot.testutil.RobotFixture;
import frc.robot.testutil.SimHarness;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** The climb on the whole simulated robot. One test: the climb cannot be undone, so nothing may follow it. */
@Tag("sim")
class ClimbMatchTest {
    private RobotContainer r;

    private void tick() {
        SimHarness.run(SimHarness.DT);
        double h = r.sim().trueCarriageIn();
        double w = r.sim().trueWristDeg();
        assertTrue(
                h >= -0.1 && w >= -91.5 && KeepOut.isLegal(Math.max(0, h), Math.max(-90, w)),
                "keep-out broken at h=" + h + " w=" + w);
        if (r.sim().trayDeg() > 0) {
            assertEquals(0, w, 8, "the wrist has to be level once the tray moves");
            assertEquals(1.25, h, 1.0);
        }
        if (Math.abs(r.sim().trueForkDeg() - 90) > 1) {
            assertEquals(90, r.sim().trayDeg(), 0.1, "the fork may not move until the tray is open");
        }
    }

    private void runUntil(State s, double seconds) {
        for (int i = 0; i < seconds / SimHarness.DT && r.climber().state() != s; i++) {
            tick();
        }
        assertEquals(s, r.climber().state());
    }

    @Test
    void climbsFromScoringPositionWithAnAlgaeAboard() {
        r = RobotFixture.reset(new Pose2d(2.5, 6.5, Rotation2d.kZero), false);
        var climb = ClimbCommands.climb(r);
        var scheduler = CommandScheduler.getInstance();

        // A short press, let go before anything has fired, puts the mechanism back in stow.
        scheduler.schedule(climb);
        SimHarness.run(0.1);
        assertEquals(Wanted.CLIMB, r.superstructure().wanted());
        climb.cancel();
        SimHarness.run(1.5);
        assertEquals(Wanted.STOW, r.superstructure().wanted());
        assertFalse(r.climber().committed());
        assertEquals(0, r.sim().trayDeg());

        // From a raised mechanism holding an algae: the algae is dropped, and the climb goes on.
        r.sim().giveAlgae();
        SimHarness.run(0.2);
        r.superstructure().setWanted(Wanted.ALGAE_LOW);
        SimHarness.run(2.0);
        scheduler.schedule(climb);
        runUntil(State.DEPLOYED, 12);
        assertFalse(r.superstructure().hasAlgae());
        assertEquals(0, r.sim().trueForkDeg(), 4);

        // Nothing the driver presses moves the mechanism now.
        climb.cancel();
        SimHarness.run(0.1);
        r.superstructure().setWanted(Wanted.L4);
        for (int i = 0; i < 25; i++) {
            tick();
        }
        assertEquals(Wanted.CLIMB, r.superstructure().wanted());

        // Back onto the cage, press again, and up.
        r.sim().setCageCaptured(true);
        scheduler.schedule(climb);
        runUntil(State.CLIMBED, 6);
        climb.cancel();
        for (int i = 0; i < 25; i++) {
            tick();
        }
        assertEquals(88, r.sim().trueForkDeg(), 5);
    }
}
