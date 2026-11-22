package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Turns a CANrange distance into a debounced "game piece present" flag. A piece is present once the
 * range has read under {@link #PRESENT_BELOW_M} for {@link #DEBOUNCE_S}, and absent once it has read
 * above it for as long.
 *
 * <p>A sensor that is not answering raises an alert and two things follow. {@link #present()} reads
 * absent, so nothing waits forever on a piece it cannot see. {@link #believedPresent()} keeps the last
 * reading from before the dropout, so an interlock that depends on a piece being aboard (the algae
 * rule, the hold voltage) does not switch off just because the sensor went quiet; only an online
 * reading or {@link #forget()} changes it.
 */
public final class PieceSensor {
    public static final double PRESENT_BELOW_M = 0.05;
    public static final double DEBOUNCE_S = 0.06;

    private final Debouncer debouncer = new Debouncer(DEBOUNCE_S, DebounceType.kBoth);
    private final Alert offlineAlert;
    private boolean online = false;
    private boolean present = false;
    private boolean believed = false;

    /** @param offlineText alert text raised while the sensor is not answering */
    public PieceSensor(String offlineText) {
        offlineAlert = new Alert(offlineText, AlertType.kError);
    }

    /** Feed one sample; returns whether a piece is reported present after it. Call once per loop. */
    public boolean update(boolean connected, double rangeM) {
        online = connected;
        offlineAlert.set(!connected);
        boolean raw = connected && Double.isFinite(rangeM) && rangeM < PRESENT_BELOW_M;
        boolean debounced = debouncer.calculate(raw);
        present = connected && debounced;
        if (connected) {
            believed = present;
        }
        return present;
    }

    /** Reported state: a piece seen by an answering sensor. False whenever the sensor is offline. */
    public boolean present() {
        return present;
    }

    /** Last known state: the reported state while online, the last online reading while offline. */
    public boolean believedPresent() {
        return believed;
    }

    public boolean online() {
        return online;
    }

    /** The piece is known to have left (pushed out long enough) though the sensor could not see it. */
    public void forget() {
        believed = present;
    }

    /** Release the alert so a discarded sensor does not leave it raised. */
    public void close() {
        offlineAlert.close();
    }
}
