package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.Ports;
import java.util.HashSet;
import org.junit.jupiter.api.Test;

class PortsTest {
    @Test
    void idsAreUniqueAcrossBothBuses() {
        var ids = Ports.all().stream().map(d -> d.can().id()).toList();
        assertEquals(ids.size(), new HashSet<>(ids).size());
    }

    @Test
    void idsFitCtreRange() {
        Ports.all().forEach(d -> assertTrue(d.can().id() >= 0 && d.can().id() <= 62));
    }

    @Test
    void deviceCounts() {
        assertEquals(23, Ports.all().size());
        assertEquals(
                13,
                Ports.all().stream().filter(d -> d.can().bus().equals(Ports.DRIVE_BUS)).count());
        assertEquals(
                10,
                Ports.all().stream().filter(d -> d.can().bus().equals(Ports.RIO_BUS)).count());
    }

    @Test
    void reservedIdsAreFree() {
        Ports.all().forEach(d -> assertFalse(d.can().id() >= 60));
    }
}
