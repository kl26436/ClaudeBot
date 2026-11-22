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

    /**
     * @throws IllegalArgumentException if either measurement is NaN
     */
    public static Targets next(double carriageIn, double wristDeg, Targets goal, boolean hasAlgae) {
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
        double wristTarget = clamp(goalWrist, range.min(), range.max());
        // The keep-out table outranks the algae rule: only floor the wrist where the zone range reaches the floor.
        if (hasAlgae && range.max() >= KeepOut.ALGAE_MIN_WRIST_DEG) {
            wristTarget = Math.max(wristTarget, KeepOut.ALGAE_MIN_WRIST_DEG);
        }

        // Carriage target: legal for the measured wrist and for the wrist being commanded. The measured
        // wrist is judged MEASURED_MARGIN_DEG lower than it reads (every restricted band limits the
        // low side), so a wrist still settling toward its target does not let the carriage into a
        // band it is barely clear of.
        double wJudged = Math.max(WristK.MIN_DEG, w - MEASURED_MARGIN_DEG);
        KeepOut.Range now = pulledInSpan(h, wJudged);
        KeepOut.Range next = pulledInSpan(h, wristTarget);
        double lo = Math.max(now.min(), next.min());
        double hi = Math.min(now.max(), next.max());
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

    private static KeepOut.Range marginedRange(int zoneIndex) {
        KeepOut.Zone z = KeepOut.ZONES.get(zoneIndex);
        double lo = z.minWristDeg() > WristK.MIN_DEG ? z.minWristDeg() + MARGIN_DEG : z.minWristDeg();
        return new KeepOut.Range(lo, Math.max(lo, z.maxWristDeg()));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
