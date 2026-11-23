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
                var t = SuperstructurePlanner.next(h, 0, w, 0, goal, false);
                assertTrue(KeepOut.isLegal(t.carriageIn(), t.wristDeg()),
                        from + " to " + to + " planner output " + t + " from h=" + h + " w=" + w);
                h += MathUtil.clamp(t.carriageIn() - h, -1.1, 1.1);
                w += MathUtil.clamp(t.wristDeg() - w, -4, 4);
                assertTrue(KeepOut.isLegal(h, w), from + " to " + to + " at h=" + h + " w=" + w);
                assertTrue(++ticks < 2000, from + " to " + to + " did not finish");
            }
        }
    }

    @Test void stowToL2WaitsAtTheUnstowHeightUntilTheWristIsUp() {
        var t = SuperstructurePlanner.next(0, 0, -90, 0, new Targets(17.5, -35), false);
        assertEquals(1.25, t.carriageIn(), 1e-9);
        assertEquals(-90, t.wristDeg(), 1e-9);
        var at = SuperstructurePlanner.next(1.25, 0, -90, 0, new Targets(17.5, -35), false);
        assertEquals(1.25, at.carriageIn(), 1e-9, "a stowed wrist above the band would meet the climber fork");
        assertEquals(-35, at.wristDeg(), 1e-9);
    }

    @Test void stowingStopsAtTheUnstowHeightUntilTheWristIsStowed() {
        var t = SuperstructurePlanner.next(10, 0, -35, 0, new Targets(0, -90), false);
        assertEquals(1.25, t.carriageIn(), 1e-9);
        var at = SuperstructurePlanner.next(1.25, 0, -35, 0, new Targets(0, -90), false);
        assertEquals(1.25, at.carriageIn(), 1e-9);
        assertEquals(-90, at.wristDeg(), 1e-9);
        assertEquals(0, SuperstructurePlanner.next(1.25, 0, -90, 0, new Targets(0, -90), false).carriageIn(), 1e-9);
    }

    @Test void theWristHeadsForTheTightestBandOnTheWayFromTheStart() {
        // L4's -52 is illegal from 9 to 23 in, so the wrist goes to the 15 to 23 in limit at once.
        var t = SuperstructurePlanner.next(1.25, 0, -90, 0, new Targets(65.25, -52), false);
        assertEquals(1.25, t.carriageIn(), 1e-9);
        assertEquals(-37, t.wristDeg(), 1e-9);
        var below9 = SuperstructurePlanner.next(8.5, 0, -52, 0, new Targets(65.25, -52), false);
        assertEquals(8.5, below9.carriageIn(), 1e-9, "held until the wrist is inside the next band's limit");
        assertEquals(-37, below9.wristDeg(), 1e-9);
        assertEquals(-52, SuperstructurePlanner.next(30, 0, -37, 0, new Targets(65.25, -52), false).wristDeg(), 1e-9);
    }

    @Test void algaeNeverDropsBelowTen() {
        var t = SuperstructurePlanner.next(8.8, 0, 46, 0, new Targets(0, -90), true);
        assertTrue(t.wristDeg() >= 10);
        assertTrue(t.carriageIn() >= 1.25);
    }

    @Test void stowedWristReadingJustUnderMinus90StillLeavesStow() {
        var t = SuperstructurePlanner.next(0, 0, -90.5, 0, new Targets(17.5, -35), false);
        assertEquals(1.25, t.carriageIn(), 1e-9);
        assertEquals(-90, t.wristDeg(), 1e-9);
    }

    /**
     * The intake pose sits 0.25 in above the stow-only zone. A carriage that dips into it must rise
     * again with the wrist still: swinging the wrist down to stow there would hit the lift gearbox.
     */
    @Test void aCarriageThatDipsBelowTheBandRisesWithTheWristHeld() {
        for (var goal : new Targets[] {new Targets(1.25, -35), new Targets(0, -90), new Targets(17.5, -35)}) {
            for (double w : new double[] {-74, -65, -55, -45, -35, 0, 40}) {
                var t = SuperstructurePlanner.next(0.6, 0, w, 0, goal, false);
                assertEquals(1.25, t.carriageIn(), 1e-9, "w=" + w + " goal " + goal);
                assertTrue(t.wristDeg() >= Math.min(w, -40) - 1e-9, "w=" + w + " goal " + goal + " sent to " + t.wristDeg());
            }
        }
        // Barely out of stow is the one case where stow is the nearer safe place.
        assertEquals(-90, SuperstructurePlanner.next(0.5, 0, -86, 0, new Targets(0, -90), false).wristDeg(), 1e-9);
    }

    @Test void aCarriageSettlingOntoTheIntakeHeightDoesNotSendTheWristToStow() {
        // Slowing down onto 1.25 in, one loop of latency reaches below the band.
        var t = SuperstructurePlanner.next(1.7, -35, -35, 0, new Targets(1.25, -35), false);
        assertEquals(-35, t.wristDeg(), 1e-9);
    }

    /**
     * A real axis settles a little off its target (friction, an imperfect gravity term), and no move may
     * wait for ever on the last of it: every move must still finish with each axis anywhere inside its
     * own tolerance.
     */
    @Test void movesFinishWithEachAxisSettlingOffItsTarget() {
        for (var from : Setpoints.Pose.values()) for (var to : Setpoints.Pose.values()) {
            for (double eh : new double[] {-0.45, 0, 0.45}) for (double ew : new double[] {-1.4, 0, 1.4}) {
                double h = from.carriageIn, w = from.wristDeg;
                var goal = new Targets(to.carriageIn, to.wristDeg);
                int ticks = 0;
                while (Math.abs(h - goal.carriageIn()) > Setpoints.CARRIAGE_TOL_IN || Math.abs(w - goal.wristDeg()) > Setpoints.WRIST_TOL_DEG) {
                    var t = SuperstructurePlanner.next(h, 0, w, 0, goal, false);
                    h += MathUtil.clamp(Math.max(0, t.carriageIn() + eh) - h, -1.1, 1.1);
                    w += MathUtil.clamp(t.wristDeg() + ew - w, -4, 4);
                    String at = from + " to " + to + " settling " + eh + " in, " + ew + " deg off, at h=" + h + " w=" + w;
                    assertTrue(KeepOut.isLegal(MathUtil.clamp(h, 0, 65.25), MathUtil.clamp(w, -90, 75)), at);
                    assertTrue(++ticks < 2000, "stuck: " + at);
                }
            }
        }
    }

    /**
     * One axis following its target the way Motion Magic does: time-optimal under a cruise speed and
     * an acceleration limit. Returns {position, velocity} after dt.
     */
    private static double[] follow(double p, double v, double target, double maxV, double maxA, double dt) {
        double err = target - p;
        double want = Math.signum(err) * Math.min(maxV, Math.sqrt(2 * maxA * Math.abs(err)));
        double nv = v + MathUtil.clamp(want - v, -maxA * dt, maxA * dt);
        double np = p + (v + nv) / 2 * dt;
        if (Math.abs(target - np) < 1e-3 && Math.abs(nv) < 2 * maxA * dt) {
            return new double[] {target, 0};
        }
        return new double[] {np, nv};
    }

    /**
     * Change the goal part-way through a move: A to B, then C at a chosen moment. The carriage cannot
     * stop at once (up to 9 in from full speed), so a planner that only looks at where the carriage
     * is now lets the wrist into a band the carriage is still sliding into. 20 ms loop, the targets
     * one loop late, each axis limited by its Motion Magic cruise and acceleration.
     */
    @Test void interruptedMovesStayLegal() {
        interruptedMoves(Setpoints.Pose.values(), false);
    }

    @Test void interruptedMovesWithAnAlgaeStayLegal() {
        interruptedMoves(
                new Setpoints.Pose[] {
                    Setpoints.Pose.ALGAE_LOW, Setpoints.Pose.ALGAE_HIGH, Setpoints.Pose.ALGAE_CARRY, Setpoints.Pose.NET,
                    Setpoints.Pose.PROCESSOR
                },
                true);
    }

    private static void interruptedMoves(Setpoints.Pose[] poses, boolean hasAlgae) {
        double cruiseIn = frc.robot.config.RobotConstants.ElevatorK.CRUISE_IN_PER_S;
        double accelIn = frc.robot.config.RobotConstants.ElevatorK.ACCEL_IN_PER_S2;
        double cruiseDeg = frc.robot.config.RobotConstants.WristK.CRUISE_DEG_PER_S;
        double accelDeg = frc.robot.config.RobotConstants.WristK.ACCEL_DEG_PER_S2;
        int worstCount = 0;
        String worst = "";
        int unfinished = 0;
        String firstUnfinished = "";
        for (var a : poses) for (var b : poses) for (var c : poses) {
            if (a == b || b == c) continue;
            for (int cut = 1; cut < 100; cut += 3) {
                double h = a.carriageIn, w = a.wristDeg, vh = 0, vw = 0;
                var goal = new Targets(b.carriageIn, b.wristDeg);
                var pending = new Targets(h, w);
                for (int tick = 0; tick < 250; tick++) {
                    if (tick == cut) goal = new Targets(c.carriageIn, c.wristDeg);
                    var sent = pending;
                    pending = SuperstructurePlanner.next(h, vh, w, vw, goal, hasAlgae);
                    for (int k = 0; k < 10; k++) {
                        double[] hv = follow(h, vh, sent.carriageIn(), cruiseIn, accelIn, 0.002);
                        double[] wv = follow(w, vw, sent.wristDeg(), cruiseDeg, accelDeg, 0.002);
                        h = hv[0]; vh = hv[1]; w = wv[0]; vw = wv[1];
                        double hc = MathUtil.clamp(h, 0, 65.25), wc = MathUtil.clamp(w, -90, 75);
                        if (!KeepOut.isLegal(hc, wc) || (hasAlgae && w < KeepOut.ALGAE_MIN_WRIST_DEG - 0.5)) {
                            worstCount++;
                            if (worst.isEmpty()) {
                                worst = a + " to " + b + ", then " + c + " at " + cut * 20 + " ms: h=" + h + " w=" + w;
                            }
                            tick = 250;
                            break;
                        }
                    }
                }
                if (Math.abs(h - c.carriageIn) > 0.1 || Math.abs(w - c.wristDeg) > 0.1) {
                    unfinished++;
                    if (firstUnfinished.isEmpty()) {
                        firstUnfinished = a + " to " + b + ", then " + c + " at " + cut * 20 + " ms: stuck at h=" + h + " w=" + w;
                    }
                }
            }
        }
        assertEquals(0, worstCount, "interrupted moves that broke keep-out; first: " + worst);
        assertEquals(0, unfinished, "interrupted moves that never reached the new goal; first: " + firstUnfinished);
    }

    @Test void aCarriageSlidingDownHoldsTheWristForTheBandItWillReach() {
        // At 30 in, falling at 65 in/s: it cannot stop above 23 in, so L4's -52 must wait.
        var t = SuperstructurePlanner.next(30, -65, -35, 0, new Targets(65.25, -52), false);
        assertTrue(t.wristDeg() >= -37, "wrist sent to " + t.wristDeg() + " while the carriage slides into the funnel band");
        var stopped = SuperstructurePlanner.next(30, 0, -35, 0, new Targets(65.25, -52), false);
        assertEquals(-52, stopped.wristDeg(), 1e-9, "at rest above 23 in, -52 is fine");
    }

    @Test void aWristBarelyClearOfTheFunnelDoesNotReleaseTheCarriage() {
        // -39.5 is legal above 15 in (limit -40), but with no margin a little lag puts it inside.
        var t = SuperstructurePlanner.next(14.5, 0, -39.5, 0, new Targets(17.5, -35), false);
        assertTrue(t.carriageIn() <= 14.5, "carriage released into the funnel band at " + t.carriageIn());
        assertEquals(-35, t.wristDeg(), 1e-9);
    }

    @Test void aWristSettledInsideTheMarginReleasesTheCarriage() {
        var t = SuperstructurePlanner.next(14.5, 0, -38.5, 0, new Targets(17.5, -35), false);
        assertEquals(17.5, t.carriageIn(), 1e-9);
    }

    @Test void aWristSaggingInTheFunnelBandHoldsTheCarriage() {
        var t = SuperstructurePlanner.next(17, 0, -39.5, 0, new Targets(0, -90), false);
        assertEquals(17, t.carriageIn(), 1e-9);
    }

    @Test void illegalStateHoldsTheCarriage() {
        var t = SuperstructurePlanner.next(17, 0, -60, 0, new Targets(30, -35), false);
        assertEquals(17, t.carriageIn(), 1e-9);
        assertEquals(-35, t.wristDeg(), 1e-9);
    }

    @Test void nanMeasurementThrows() {
        assertThrows(IllegalArgumentException.class,
                () -> SuperstructurePlanner.next(Double.NaN, 0, 0, 0, new Targets(5, 0), false));
        assertThrows(IllegalArgumentException.class,
                () -> SuperstructurePlanner.next(5, 0, Double.NaN, 0, new Targets(5, 0), false));
    }

    @Test void algaeFlagNeverPullsTheWristOutOfStow() {
        var t = SuperstructurePlanner.next(0, 0, -90, 0, new Targets(17.5, -35), true);
        assertEquals(-90, t.wristDeg(), 1e-9);
        assertEquals(1.25, t.carriageIn(), 1e-9);
    }

    @Test void algaeWithStowGoalKeepsTheCarriageAboveTheStowZone() {
        var t = SuperstructurePlanner.next(1.3, 0, -90, 0, new Targets(0, -90), true);
        assertEquals(10, t.wristDeg(), 1e-9);
        assertEquals(1.25, t.carriageIn(), 1e-9);
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
                var t = SuperstructurePlanner.next(h, 0, w, 0, goal, true);
                assertTrue(KeepOut.isLegal(t.carriageIn(), t.wristDeg()),
                        from + " to " + to + " tick " + tick + " planner output " + t + " from h=" + h + " w=" + w);
                h += MathUtil.clamp(t.carriageIn() - h, -1.1, 1.1);
                w += MathUtil.clamp(t.wristDeg() - w, -4, 4);
                String at = from + " to " + to + " tick " + tick + " h=" + h + " w=" + w;
                assertTrue(KeepOut.isLegal(h, w), at);
                assertTrue(w >= KeepOut.ALGAE_MIN_WRIST_DEG - 1e-9, at);
                assertTrue(h >= KeepOut.UNSTOW_MIN_IN - 1e-9, at);
            }
        }
    }
}
