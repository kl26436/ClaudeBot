package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PieceSensor;
import org.littletonrobotics.junction.Logger;

/** Coral rollers. {@link #intake()} runs inward and switches itself to {@link #hold()} once a coral is seen. */
public class CoralRollers extends SubsystemBase {
    static final double INTAKE_VOLTS = 4; // PLACEHOLDER(tuning)
    static final double HOLD_VOLTS = 0.5; // PLACEHOLDER(tuning)
    static final double EJECT_VOLTS = -6; // PLACEHOLDER(tuning)

    /** Pushing out for this long clears a piece the sensor cannot see. */
    static final double EJECT_CLEARS_S = 0.3; // PLACEHOLDER(tuning)

    private enum Mode {
        STOPPED,
        INTAKE,
        HOLD,
        EJECT
    }

    private final CoralRollersIO io;
    private final CoralRollersIOInputsAutoLogged inputs = new CoralRollersIOInputsAutoLogged();
    private final PieceSensor sensor = new PieceSensor("Coral sensor offline");
    private Mode mode = Mode.STOPPED;
    private double ejectStartS = Double.NaN;

    public CoralRollers(CoralRollersIO io) {
        this.io = io;
    }

    /** Run inward until a coral is seen, then hold it. */
    public void intake() {
        mode = sensor.present() ? Mode.HOLD : Mode.INTAKE;
    }

    public void hold() {
        mode = Mode.HOLD;
    }

    /** Push the coral out. Keeps running until another mode is chosen. */
    public void eject() {
        if (mode != Mode.EJECT) {
            ejectStartS = Timer.getFPGATimestamp();
        }
        mode = Mode.EJECT;
    }

    public void stop() {
        mode = Mode.STOPPED;
    }

    /** True while a coral is seated. False whenever the sensor is offline. */
    public boolean hasCoral() {
        return sensor.present();
    }

    /** True while the CANrange is answering. */
    public boolean sensorOnline() {
        return sensor.online();
    }

    /**
     * Whether a coral is aboard as far as anyone knows: what the sensor reports while it answers, and
     * the last answer while it does not, until a push out long enough to clear it.
     */
    public boolean mayHaveCoral() {
        return sensor.believedPresent();
    }

    public boolean isEjecting() {
        return mode == Mode.EJECT;
    }

    public void close() {
        sensor.close();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralRollers", inputs);

        boolean present = sensor.update(inputs.rangeConnected, inputs.rangeM);
        if (mode == Mode.INTAKE && present) {
            mode = Mode.HOLD;
        }
        // With the sensor offline, only a long enough push out says the piece has gone.
        if (mode == Mode.EJECT && !sensor.online() && Timer.getFPGATimestamp() - ejectStartS >= EJECT_CLEARS_S) {
            sensor.forget();
        }
        io.setVolts(switch (mode) {
            case STOPPED -> 0;
            case INTAKE -> INTAKE_VOLTS;
            case HOLD -> HOLD_VOLTS;
            case EJECT -> EJECT_VOLTS;
        });

        Logger.recordOutput("CoralRollers/Mode", mode);
        Logger.recordOutput("CoralRollers/HasCoral", present);
    }
}
