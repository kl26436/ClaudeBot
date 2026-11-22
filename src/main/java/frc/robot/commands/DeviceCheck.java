package frc.robot.commands;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.config.Ports;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Calls every device in {@link Ports#all()} and lists the ones that do not answer, by name, so a
 * wiring fault shows on the dashboard at start-up instead of as a mechanism that will not move.
 */
public final class DeviceCheck {
    private DeviceCheck() {}

    /** Total time every Phoenix device together gets to be heard from. */
    static final double ANSWER_TIMEOUT_S = 0.5;

    private static final long POLL_MS = 20;

    /** One probe handle per device, made once and kept: Phoenix shares them with the subsystems' handles. */
    private static final Map<Ports.Device, ParentDevice> PROBES = new LinkedHashMap<>();

    private static PowerDistribution pdh;

    /** Names of the devices that did not answer. Empty when the wiring matches the map. */
    public static synchronized List<String> missing() {
        List<String> missing = new ArrayList<>();
        List<Ports.Device> phoenix = new ArrayList<>();
        for (Ports.Device d : Ports.all()) {
            if (d.kind().equals("PDH")) {
                if (!pdhAnswers(d)) {
                    missing.add(d.name());
                }
            } else {
                PROBES.computeIfAbsent(d, DeviceCheck::probe);
                phoenix.add(d);
            }
        }
        long deadline = System.nanoTime() + (long) (ANSWER_TIMEOUT_S * 1e9);
        List<Ports.Device> silent = new ArrayList<>(phoenix);
        while (true) {
            silent.removeIf(d -> PROBES.get(d).isConnected());
            if (silent.isEmpty() || System.nanoTime() >= deadline) {
                break;
            }
            sleep();
        }
        for (Ports.Device d : phoenix) { // keep the Ports order in the report
            if (silent.contains(d)) {
                missing.add(d.name());
            }
        }
        return missing;
    }

    private static boolean pdhAnswers(Ports.Device d) {
        try {
            if (pdh == null) {
                pdh = new PowerDistribution(d.can().id(), ModuleType.kRev);
            }
            return true;
        } catch (RuntimeException e) {
            return false;
        }
    }

    private static void sleep() {
        try {
            Thread.sleep(POLL_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private static ParentDevice probe(Ports.Device d) {
        int id = d.can().id();
        CANBus bus = new CANBus(d.can().bus());
        return switch (d.kind()) {
            case "TalonFX" -> new TalonFX(id, bus);
            case "CANcoder" -> new CANcoder(id, bus);
            case "Pigeon2" -> new Pigeon2(id, bus);
            case "CANdi" -> new CANdi(id, bus);
            case "CANrange" -> new CANrange(id, bus);
            default -> throw new IllegalArgumentException("no probe for a " + d.kind() + ": " + d.name());
        };
    }
}
