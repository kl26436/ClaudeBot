package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import frc.robot.config.KeepOut;
import frc.robot.config.Setpoints;
import frc.robot.subsystems.SuperstructurePlanner;
import frc.robot.subsystems.SuperstructurePlanner.Targets;
import org.junit.jupiter.api.Test;

class SuperstructurePlannerTest {
    @Test void everyMoveStaysLegalAndFinishes() {
        for (var from : Setpoints.Pose.values()) for (var to : Setpoints.Pose.values()) {
            double h = from.carriageIn, w = from.wristDeg;
            var goal = new Targets(to.carriageIn, to.wristDeg);
            int ticks = 0;
            while (Math.abs(h - goal.carriageIn()) > 0.01 || Math.abs(w - goal.wristDeg()) > 0.01) {
                var t = SuperstructurePlanner.next(h, w, goal, false);
                assertTrue(KeepOut.isLegal(t.carriageIn(), t.wristDeg()),
                        from + " to " + to + " planner output " + t + " from h=" + h + " w=" + w);
                h += MathUtil.clamp(t.carriageIn() - h, -1.1, 1.1);
                w += MathUtil.clamp(t.wristDeg() - w, -4, 4);
                assertTrue(KeepOut.isLegal(h, w), from + " to " + to + " at h=" + h + " w=" + w);
                assertTrue(++ticks < 2000, from + " to " + to + " did not finish");
            }
        }
    }

    @Test void stowToL2WaitsBelowSixUntilWristIsUp() {
        var t = SuperstructurePlanner.next(0, -90, new Targets(17.5, -35), false);
        assertEquals(5.5, t.carriageIn(), 1e-9);
        assertEquals(-90, t.wristDeg(), 1e-9);
    }

    @Test void stowToL4TurnsTheWristThroughTheFunnelZoneFirst() { // L4's -52 is illegal between 6 and 23 in
        var t = SuperstructurePlanner.next(5.5, -90, new Targets(65.25, -52), false);
        assertEquals(-35, t.wristDeg(), 1e-9);
        assertEquals(5.5, t.carriageIn(), 1e-9);
    }

    @Test void algaeNeverDropsBelowTen() {
        var t = SuperstructurePlanner.next(8.8, 46, new Targets(0, -90), true);
        assertTrue(t.wristDeg() >= 10);
        assertTrue(t.carriageIn() >= 1.75);
    }

    @Test void stowedWristReadingJustUnderMinus90StillLeavesStow() {
        var t = SuperstructurePlanner.next(0, -90.5, new Targets(17.5, -35), false);
        assertEquals(5.5, t.carriageIn(), 1e-9);
        assertEquals(-90, t.wristDeg(), 1e-9);
    }

    @Test void illegalStateHoldsTheCarriage() {
        var t = SuperstructurePlanner.next(10, -60, new Targets(30, -35), false);
        assertEquals(10, t.carriageIn(), 1e-9);
        assertEquals(-35, t.wristDeg(), 1e-9);
    }

    @Test void nanMeasurementThrows() {
        assertThrows(IllegalArgumentException.class,
                () -> SuperstructurePlanner.next(Double.NaN, 0, new Targets(5, 0), false));
        assertThrows(IllegalArgumentException.class,
                () -> SuperstructurePlanner.next(5, Double.NaN, new Targets(5, 0), false));
    }

    @Test void algaeFlagNeverPullsTheWristOutOfStow() {
        var t = SuperstructurePlanner.next(0, -90, new Targets(17.5, -35), true);
        assertEquals(-90, t.wristDeg(), 1e-9);
        assertEquals(5.5, t.carriageIn(), 1e-9);
    }

    @Test void algaeWithStowGoalKeepsTheCarriageAboveTheStowZone() {
        var t = SuperstructurePlanner.next(2, -90, new Targets(0, -90), true);
        assertEquals(10, t.wristDeg(), 1e-9);
        assertEquals(1.75, t.carriageIn(), 1e-9);
    }

    @Test void algaeMovesStayLegalAndKeepTheFloor() {
        var carry = new Setpoints.Pose[] {Setpoints.Pose.ALGAE_LOW, Setpoints.Pose.ALGAE_HIGH,
            Setpoints.Pose.ALGAE_CARRY, Setpoints.Pose.NET, Setpoints.Pose.PROCESSOR};
        var targets = new Setpoints.Pose[] {Setpoints.Pose.ALGAE_LOW, Setpoints.Pose.ALGAE_HIGH,
            Setpoints.Pose.ALGAE_CARRY, Setpoints.Pose.NET, Setpoints.Pose.PROCESSOR, Setpoints.Pose.STOW};
        for (var from : carry) for (var to : targets) {
            double h = from.carriageIn, w = from.wristDeg;
            var goal = new Targets(to.carriageIn, to.wristDeg);
            for (int tick = 0; tick < 3000; tick++) {
                var t = SuperstructurePlanner.next(h, w, goal, true);
                assertTrue(KeepOut.isLegal(t.carriageIn(), t.wristDeg()),
                        from + " to " + to + " tick " + tick + " planner output " + t + " from h=" + h + " w=" + w);
                h += MathUtil.clamp(t.carriageIn() - h, -1.1, 1.1);
                w += MathUtil.clamp(t.wristDeg() - w, -4, 4);
                String at = from + " to " + to + " tick " + tick + " h=" + h + " w=" + w;
                assertTrue(KeepOut.isLegal(h, w), at);
                assertTrue(w >= KeepOut.ALGAE_MIN_WRIST_DEG - 1e-9, at);
                assertTrue(h >= 1.25 - 1e-9, at);
            }
        }
    }
}
