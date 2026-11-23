package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.KeepOut;
import frc.robot.config.Setpoints;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class KeepOutTest {
    /** Drawing R-001, rev3: the lowest wrist angle allowed, just inside both ends of each band. */
    @ParameterizedTest
    @CsvSource({"2.01,-75", "6.99,-75", "7,-60", "8.99,-60", "9,-45", "14.99,-45", "15,-40", "22.99,-40", "23,-90", "65.25,-90"})
    void lowestAngleMatchesTheDrawing(double h, double lowest) {
        assertTrue(KeepOut.isLegal(h, lowest));
        assertTrue(KeepOut.isLegal(h, 75));
        if (lowest > -90) {
            assertFalse(KeepOut.isLegal(h, lowest - 0.1));
        }
    }

    @Test void stowOnlyAtTheBottom() {
        assertTrue(KeepOut.isLegal(0, -90));
        assertFalse(KeepOut.isLegal(0.5, -60));
        assertFalse(KeepOut.isLegal(0.74, -35));
    }

    @Test void anyAngleInTheUnstowBand() {
        assertTrue(KeepOut.UNSTOW_MIN_IN < KeepOut.UNSTOW_IN && KeepOut.UNSTOW_IN < KeepOut.UNSTOW_MAX_IN);
        for (double h : new double[] {KeepOut.UNSTOW_MIN_IN, KeepOut.UNSTOW_IN, KeepOut.UNSTOW_MAX_IN - 0.01}) {
            for (double w = -90; w <= 75; w += 5) {
                assertTrue(KeepOut.isLegal(h, w), "h=" + h + " w=" + w);
            }
        }
        assertFalse(KeepOut.isLegal(KeepOut.UNSTOW_MAX_IN, -90), "a stowed wrist above the band meets the climber fork");
    }

    @Test void everySetpointIsLegal() {
        for (var p : Setpoints.Pose.values()) assertTrue(KeepOut.isLegal(p.carriageIn, p.wristDeg), p.name());
    }

    @Test void stowedWristCanReachOnlyTheTopOfTheUnstowBand() {
        var r = KeepOut.carriageRangeAt(0, -90);
        assertEquals(0, r.min(), 1e-9);
        assertEquals(KeepOut.UNSTOW_MAX_IN, r.max(), 1e-9);
    }

    @Test void levelWristCannotGoBelowTheUnstowBand() {
        var r = KeepOut.carriageRangeAt(30, 0);
        assertEquals(KeepOut.UNSTOW_MIN_IN, r.min(), 1e-9);
        assertEquals(65.25, r.max(), 1e-9);
    }

    @Test void steepWristAboveFunnelStaysAbove23() { assertEquals(23, KeepOut.carriageRangeAt(40, -60).min(), 1e-9); }

    @Test void stowToleranceAppliesOnlyInStowZone() {
        assertTrue(KeepOut.isLegal(0.5, -87.5));
        assertFalse(KeepOut.isLegal(0.5, -86.5));
    }

    @Test void outOfTravelIsIllegal() {
        assertFalse(KeepOut.isLegal(-0.1, -90));
        assertFalse(KeepOut.isLegal(65.3, 0));
        assertTrue(KeepOut.isLegal(65.25, 0));
        assertTrue(KeepOut.isLegal(0, -90));
    }

    @Test void zonesAreContiguousAndIndexClamps() {
        for (int i = 0; i + 1 < KeepOut.ZONES.size(); i++) {
            assertEquals(KeepOut.ZONES.get(i).maxCarriageIn(), KeepOut.ZONES.get(i + 1).minCarriageIn());
            assertEquals(i + 1, KeepOut.zoneIndexAt(KeepOut.ZONES.get(i + 1).minCarriageIn()));
        }
        int last = KeepOut.ZONES.size() - 1;
        assertEquals(0, KeepOut.zoneIndexAt(-5));
        assertEquals(last, KeepOut.zoneIndexAt(65.25));
        assertEquals(last, KeepOut.zoneIndexAt(100));
    }

    @Test void wristRangeAtReportsZoneRange() {
        assertEquals(-90, KeepOut.wristRangeAt(0.5).min(), 1e-9);
        assertEquals(-90, KeepOut.wristRangeAt(0.5).max(), 1e-9);
        assertEquals(-45, KeepOut.wristRangeAt(10).min(), 1e-9);
        assertEquals(75, KeepOut.wristRangeAt(10).max(), 1e-9);
    }
}
