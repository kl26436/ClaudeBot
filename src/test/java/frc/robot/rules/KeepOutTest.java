package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.KeepOut;
import frc.robot.config.Setpoints;
import org.junit.jupiter.api.Test;

class KeepOutTest {
    @Test void stowOnlyAtTheBottom() { assertTrue(KeepOut.isLegal(0, -90)); assertFalse(KeepOut.isLegal(0.5, -60)); }
    @Test void anyAngleBetween1_25And6() { assertTrue(KeepOut.isLegal(3, -90)); assertTrue(KeepOut.isLegal(3, 75)); }
    @Test void funnelZoneLimit() { assertFalse(KeepOut.isLegal(10, -40)); assertTrue(KeepOut.isLegal(10, -38)); }
    @Test void anyAngleAbove23() { assertTrue(KeepOut.isLegal(40, -90)); }
    @Test void everySetpointIsLegal() {
        for (var p : Setpoints.Pose.values()) assertTrue(KeepOut.isLegal(p.carriageIn, p.wristDeg), p.name());
    }
    @Test void stowedWristCanReachOnlyUpTo6() {
        var r = KeepOut.carriageRangeAt(0, -90);
        assertEquals(0, r.min(), 1e-9);
        assertEquals(6, r.max(), 1e-9);
    }
    @Test void levelWristCannotGoBelow1_25() {
        var r = KeepOut.carriageRangeAt(30, 0);
        assertEquals(1.25, r.min(), 1e-9);
        assertEquals(65.25, r.max(), 1e-9);
    }
    @Test void steepWristAboveFunnelStaysAbove23() { assertEquals(23, KeepOut.carriageRangeAt(40, -60).min(), 1e-9); }

    @Test void stowToleranceAppliesOnlyInStowZone() {
        assertTrue(KeepOut.isLegal(0.5, -88.5));
        assertFalse(KeepOut.isLegal(0.5, -87));
    }
    @Test void outOfTravelIsIllegal() {
        assertFalse(KeepOut.isLegal(-0.1, -90));
        assertFalse(KeepOut.isLegal(65.3, 0));
        assertTrue(KeepOut.isLegal(65.25, 0));
        assertTrue(KeepOut.isLegal(0, -90));
    }
    @Test void zoneIndexClampsAndRespectsBoundaries() {
        assertEquals(0, KeepOut.zoneIndexAt(-5));
        assertEquals(1, KeepOut.zoneIndexAt(1.25));
        assertEquals(2, KeepOut.zoneIndexAt(6));
        assertEquals(3, KeepOut.zoneIndexAt(23));
        assertEquals(3, KeepOut.zoneIndexAt(65.25));
        assertEquals(3, KeepOut.zoneIndexAt(100));
    }
    @Test void wristRangeAtReportsZoneRange() {
        assertEquals(-90, KeepOut.wristRangeAt(0.5).min(), 1e-9);
        assertEquals(-90, KeepOut.wristRangeAt(0.5).max(), 1e-9);
        assertEquals(-38, KeepOut.wristRangeAt(10).min(), 1e-9);
        assertEquals(75, KeepOut.wristRangeAt(10).max(), 1e-9);
    }
}
