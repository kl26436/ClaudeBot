package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Turns a CANrange distance into a debounced "game piece present" flag. A piece is present once the
 * range has read under {@link #PRESENT_BELOW_M} for {@link #DEBOUNCE_S}, and absent once it has read
 * above it for as long. A sensor that is not answering reads absent and raises an alert.
 */
public final class PieceSensor {
    public static final double PRESENT_BELOW_M = 0.05;
    public static final double DEBOUNCE_S = 0.06;

    private final Debouncer debouncer = new Debouncer(DEBOUNCE_S, DebounceType.kBoth);
    private final Alert offlineAlert;
    private boolean present = false;

    /** @param offlineText alert text raised while the sensor is not answering */
    public PieceSensor(String offlineText) {
        offlineAlert = new Alert(offlineText, AlertType.kError);
    }

    /** Feed one sample; returns whether a piece is present after it. Call once per loop. */
    public boolean update(boolean connected, double rangeM) {
        offlineAlert.set(!connected);
        boolean raw = connected && Double.isFinite(rangeM) && rangeM < PRESENT_BELOW_M;
        boolean debounced = debouncer.calculate(raw);
        // An offline sensor is believed at once: nothing it last said can be trusted.
        present = connected && debounced;
        return present;
    }

    public boolean present() {
        return present;
    }

    /** Release the alert so a discarded sensor does not leave it raised. */
    public void close() {
        offlineAlert.close();
    }
}
