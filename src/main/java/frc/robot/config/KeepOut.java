package frc.robot.config;

import java.util.List;

/**
 * Which wrist angles (degrees from level, stow -90) are allowed at each elevator carriage height (inches of
 * travel). Pure data and queries; no WPILib dependencies.
 */
public final class KeepOut {
    private KeepOut() {}

    /** A band of carriage heights {@code [minCarriageIn, maxCarriageIn)} and the wrist range allowed in it. */
    public record Zone(double minCarriageIn, double maxCarriageIn, double minWristDeg, double maxWristDeg) {}

    public record Range(double min, double max) {}

    public static final double STOW_TOL_DEG = 2.0;
    public static final double ALGAE_MIN_WRIST_DEG = 10.0;

    private static final double STOW_DEG = -90.0;
    private static final double EPS = 1e-9;

    /** Contiguous, ordered zones. The last zone includes its top. */
    public static final List<Zone> ZONES = List.of(
            new Zone(0, 1.25, -90, -90), // stow only
            new Zone(1.25, 6, -90, 75),
            new Zone(6, 23, -38, 75), // PLACEHOLDER(funnel-redesign) Rev A gave -35
            new Zone(23, 65.25, -90, 75));

    private static final double TOP_IN = ZONES.get(ZONES.size() - 1).maxCarriageIn();

    /** Index into {@link #ZONES} of the zone containing this height; clamped to the first/last zone outside it. */
    public static int zoneIndexAt(double carriageIn) {
        for (int i = 0; i < ZONES.size() - 1; i++) {
            if (carriageIn < ZONES.get(i).maxCarriageIn()) {
                return i;
            }
        }
        return ZONES.size() - 1;
    }

    /** Allowed wrist range of the zone at this height (min == max == -90 in the stow-only zone). */
    public static Range wristRangeAt(double carriageIn) {
        Zone z = ZONES.get(zoneIndexAt(carriageIn));
        return new Range(z.minWristDeg(), z.maxWristDeg());
    }

    public static boolean isLegal(double carriageIn, double wristDeg) {
        if (carriageIn < -EPS || carriageIn > TOP_IN + EPS) {
            return false;
        }
        return zoneAllows(zoneIndexAt(carriageIn), wristDeg);
    }

    /**
     * Largest connected span of carriage heights containing {@code carriageIn} in which {@code wristDeg} is legal.
     * If the wrist is not legal at {@code carriageIn}, returns the empty span {@code [carriageIn, carriageIn]}.
     */
    public static Range carriageRangeAt(double carriageIn, double wristDeg) {
        int idx = zoneIndexAt(carriageIn);
        if (!isLegal(carriageIn, wristDeg)) {
            return new Range(carriageIn, carriageIn);
        }
        int lo = idx;
        while (lo > 0 && zoneAllows(lo - 1, wristDeg)) {
            lo--;
        }
        int hi = idx;
        while (hi < ZONES.size() - 1 && zoneAllows(hi + 1, wristDeg)) {
            hi++;
        }
        return new Range(ZONES.get(lo).minCarriageIn(), ZONES.get(hi).maxCarriageIn());
    }

    private static boolean zoneAllows(int zoneIndex, double wristDeg) {
        Zone z = ZONES.get(zoneIndex);
        if (z.minWristDeg() == z.maxWristDeg()) { // stow-only zone
            return Math.abs(wristDeg - STOW_DEG) <= STOW_TOL_DEG + EPS;
        }
        return wristDeg >= z.minWristDeg() - EPS && wristDeg <= z.maxWristDeg() + EPS;
    }
}
