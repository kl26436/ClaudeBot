package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.Ports;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
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

    private static List<String> csv() throws IOException {
        return Files.readAllLines(Path.of("docs", "cad", "can_map.csv"), StandardCharsets.UTF_8);
    }

    /** The roboRIO port whose use in the CAD list contains this text, for example "DIO 1". */
    private static String portFor(String use) throws IOException {
        var hits = csv().stream()
                .filter(l -> l.matches("(DIO|PWM) \\d.*") && l.contains(use))
                .map(l -> l.split(",")[0])
                .toList();
        assertEquals(1, hits.size(), use);
        return hits.get(0);
    }

    /** The CAD package's CAN map, "bus kind" by id. */
    private static Map<Integer, String> cadMap() throws IOException {
        var out = new TreeMap<Integer, String>();
        for (String line : csv()) {
            String[] f = line.split(",");
            if (f.length >= 3 && f[0].matches("\\d+")) {
                out.put(Integer.parseInt(f[0]), f[1] + " " + KIND.get(f[2]));
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
    void rioPortsMatchTheCadList() throws IOException {
        assertEquals(portFor("elevator home switch"), "DIO " + Ports.ELEVATOR_HOME_DIO);
        assertEquals(portFor("wrist absolute encoder"), "DIO " + Ports.WRIST_ENCODER_DIO);
        assertEquals(portFor("funnel latch servo"), "PWM " + Ports.FUNNEL_LATCH_PWM);
        assertEquals(portFor("climber pawl servos"), "PWM " + Ports.CLIMBER_PAWL_PWM);
    }
}
