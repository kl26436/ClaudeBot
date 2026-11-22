package frc.robot.subsystems;

import frc.robot.config.KeepOut;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.RobotConstants.WristK;

/**
 * Pure planner: each control loop, turns the measured carriage height and wrist angle plus a goal into safe targets
 * for both axes, using the {@link KeepOut} table. The caller owns sensor validity.
 */
public final class SuperstructurePlanner {
    private SuperstructurePlanner() {}

    public record Targets(double carriageIn, double wristDeg) {}

    public static final double MARGIN_IN = 0.5;
    public static final double MARGIN_DEG = 3.0;

    /**
     * How much worse than it reads the measured wrist is judged when releasing the carriage. Kept
     * under {@code MARGIN_DEG - WRIST_TOL_DEG} so a wrist within tolerance of its target never
     * holds the carriage.
     */
    public static final double MEASURED_MARGIN_DEG = 1.0;

    /** One control loop: a target set now reaches the motor controllers about this much later. */
    public static final double LOOP_S = 0.02;

    /**
     * Where an axis at {@code p} moving at {@code v} comes to rest if told to stop now: one loop of
     * latency at full speed, then Motion Magic's deceleration {@code a}.
     */
    static double stoppingPoint(double p, double v, double a) {
        if (!Double.isFinite(v)) {
            return p;
        }
        return p + v * LOOP_S + v * Math.abs(v) / (2 * a);
    }

    /**
     * Targets for both axes. Each axis is judged over everywhere it will pass through before it can
     * stop, not just where it is: a carriage at full speed needs about 9 in to stop and a wrist about
     * 50 degrees, so a goal changed part-way through a move must not send either axis toward an angle
     * or height the other is still sliding past.
     *
     * @param carriageVelInPerS measured carriage velocity, positive up
     * @param wristVelDegPerS measured wrist velocity, positive up
     * @throws IllegalArgumentException if either position measurement is NaN
     */
    public static Targets next(
            double carriageIn,
            double carriageVelInPerS,
            double wristDeg,
            double wristVelDegPerS,
            Targets goal,
            boolean hasAlgae) {
        if (Double.isNaN(carriageIn) || Double.isNaN(wristDeg)) {
            throw new IllegalArgumentException(
                    "NaN measurement: carriageIn=" + carriageIn + " wristDeg=" + wristDeg);
        }
        // A real stowed wrist can read slightly outside its soft range; keep-out queries need in-range inputs.
        double h = clamp(carriageIn, 0, ElevatorK.SOFT_MAX_IN);
        double w = clamp(wristDeg, WristK.MIN_DEG, WristK.MAX_DEG);

        double goalWrist = goal.wristDeg();
        if (hasAlgae) {
            goalWrist = Math.max(goalWrist, KeepOut.ALGAE_MIN_WRIST_DEG);
        }

        // Wrist target: stay inside the margined range of this zone and, when heading to another zone, of the next one.
        int zone = KeepOut.zoneIndexAt(h);
        int goalZone = KeepOut.zoneIndexAt(goal.carriageIn());
        KeepOut.Range range = marginedRange(zone);
        if (goalZone != zone) {
            KeepOut.Range nextRange = marginedRange(zone + (goalZone > zone ? 1 : -1));
            double lo = Math.max(range.min(), nextRange.min());
            double hi = Math.min(range.max(), nextRange.max());
            // With algae, never narrow a range that reaches the floor down to one that cannot (the stow-only zone).
            boolean keepsAlgaeFloor = !hasAlgae || range.max() < KeepOut.ALGAE_MIN_WRIST_DEG
                    || hi >= KeepOut.ALGAE_MIN_WRIST_DEG;
            if (lo <= hi && keepsAlgaeFloor) {
                range = new KeepOut.Range(lo, hi);
            }
        }
        // Every zone the carriage passes through before it can stop must allow the wrist target too.
        double hStop = clamp(stoppingPoint(h, carriageVelInPerS, ElevatorK.MAX_ACCEL_IN_PER_S2), 0, ElevatorK.SOFT_MAX_IN);
        KeepOut.Range sliding = marginedRangeOver(Math.min(h, hStop), Math.max(h, hStop));
        if (sliding != null) {
            double lo = Math.max(range.min(), sliding.min());
            double hi = Math.min(range.max(), sliding.max());
            range = lo <= hi ? new KeepOut.Range(lo, hi) : sliding;
        }
        double wristTarget = clamp(goalWrist, range.min(), range.max());
        // The keep-out table outranks the algae rule: only floor the wrist where the zone range reaches the floor.
        if (hasAlgae && range.max() >= KeepOut.ALGAE_MIN_WRIST_DEG) {
            wristTarget = Math.max(wristTarget, KeepOut.ALGAE_MIN_WRIST_DEG);
        }

        // Carriage target: legal for the measured wrist and for the wrist being commanded. The measured
        // wrist is judged MEASURED_MARGIN_DEG lower than it reads (every restricted band limits the
        // low side), so a wrist still settling toward its target does not let the carriage into a
        // band it is barely clear of.
        // It is judged over its whole stopping sweep, so a wrist swinging away from a band's limit
        // does not let the carriage in on the strength of where it happens to be this loop.
        double wStop = clamp(stoppingPoint(w, wristVelDegPerS, WristK.ACCEL_DEG_PER_S2), WristK.MIN_DEG, WristK.MAX_DEG);
        double wLow = Math.max(WristK.MIN_DEG, Math.min(w, wStop) - MEASURED_MARGIN_DEG);
        double wHigh = Math.min(WristK.MAX_DEG, Math.max(w, wStop) + MEASURED_MARGIN_DEG);
        KeepOut.Range nowLow = pulledInSpan(h, wLow);
        KeepOut.Range nowHigh = pulledInSpan(h, wHigh);
        KeepOut.Range next = pulledInSpan(h, wristTarget);
        double lo = Math.max(Math.max(nowLow.min(), nowHigh.min()), next.min());
        double hi = Math.min(Math.min(nowLow.max(), nowHigh.max()), next.max());
        double carriageTarget = lo > hi ? h : clamp(goal.carriageIn(), lo, hi);

        return new Targets(carriageTarget, wristTarget);
    }

    /** Carriage span reachable from height h with the wrist at wristDeg, pulled in from every non-soft-limit end. */
    private static KeepOut.Range pulledInSpan(double h, double wristDeg) {
        KeepOut.Range span = KeepOut.carriageRangeAt(h, wristDeg);
        double lo = span.min() > 0 ? span.min() + MARGIN_IN : span.min();
        double hi = span.max() < ElevatorK.SOFT_MAX_IN ? span.max() - MARGIN_IN : span.max();
        return new KeepOut.Range(lo, hi);
    }

    /** The margined wrist range every zone from {@code loIn} to {@code hiIn} allows, or null if none does. */
    private static KeepOut.Range marginedRangeOver(double loIn, double hiIn) {
        double lo = WristK.MIN_DEG;
        double hi = WristK.MAX_DEG;
        for (int z = KeepOut.zoneIndexAt(loIn); z <= KeepOut.zoneIndexAt(hiIn); z++) {
            KeepOut.Range r = marginedRange(z);
            lo = Math.max(lo, r.min());
            hi = Math.min(hi, r.max());
        }
        return lo <= hi ? new KeepOut.Range(lo, hi) : null;
    }

    private static KeepOut.Range marginedRange(int zoneIndex) {
        KeepOut.Zone z = KeepOut.ZONES.get(zoneIndex);
        double lo = z.minWristDeg() > WristK.MIN_DEG ? z.minWristDeg() + MARGIN_DEG : z.minWristDeg();
        return new KeepOut.Range(lo, Math.max(lo, z.maxWristDeg()));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
