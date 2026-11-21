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

        // Carriage target: the span reachable with the measured wrist, pulled in from non-soft-limit ends.
        KeepOut.Range span = KeepOut.carriageRangeAt(h, w);
        double lo = span.min() > 0 ? span.min() + MARGIN_IN : span.min();
        double hi = span.max() < ElevatorK.SOFT_MAX_IN ? span.max() - MARGIN_IN : span.max();
        double carriageTarget = lo > hi ? h : clamp(goal.carriageIn(), lo, hi);

        return new Targets(carriageTarget, wristTarget);
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
