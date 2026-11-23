package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.Ports;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.Map;
import java.util.TreeMap;
import org.junit.jupiter.api.Test;

class PortsTest {
    private static final Map<String, String> KIND = Map.of(
            "REV PDH", "PDH",
            "Pigeon 2.0", "Pigeon2",
            "Kraken X60", "TalonFX",
            "Kraken X44", "TalonFX",
            "CANcoder", "CANcoder",
            "CANrange", "CANrange");

    /** The CAD package's CAN map, "bus kind" by id. */
    private static Map<Integer, String> cadMap() throws IOException {
        var out = new TreeMap<Integer, String>();
        try (var in = PortsTest.class.getResourceAsStream("/robotA_can_map.csv")) {
            for (String line : new String(in.readAllBytes(), StandardCharsets.UTF_8).split("\r?\n")) {
                String[] f = line.split(",");
                if (f.length >= 3 && f[0].matches("\\d+")) {
                    out.put(Integer.parseInt(f[0]), f[1] + " " + KIND.get(f[2]));
                }
            }
        }
        return out;
    }

    @Test
    void everyDeviceMatchesTheCadCanMap() throws IOException {
        var code = new TreeMap<Integer, String>();
        Ports.all().forEach(d -> code.put(d.can().id(), d.can().bus() + " " + d.kind()));
        assertEquals(24, cadMap().size(), "the list itself was read");
        assertEquals(cadMap(), code);
    }

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
    void rioPortsMatchTheCadList() {
        assertEquals(0, Ports.ELEVATOR_HOME_DIO);
        assertEquals(1, Ports.WRIST_ENCODER_DIO);
        assertEquals(0, Ports.FUNNEL_LATCH_PWM);
        assertEquals(1, Ports.CLIMBER_PAWL_PWM);
    }
}
