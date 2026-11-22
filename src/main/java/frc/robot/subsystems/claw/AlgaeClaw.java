package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PieceSensor;
import org.littletonrobotics.junction.Logger;

/** Algae claw. {@link #intake()} runs inward and switches itself to {@link #hold()} once an algae is seen. */
public class AlgaeClaw extends SubsystemBase {
    static final double INTAKE_VOLTS = 6; // PLACEHOLDER(tuning)
    static final double HOLD_VOLTS = 1.5; // PLACEHOLDER(tuning)
    static final double EJECT_VOLTS = -8; // PLACEHOLDER(tuning)

    /** Pushing out for this long clears a piece the sensor cannot see. */
    static final double EJECT_CLEARS_S = 0.3; // PLACEHOLDER(tuning)

    private enum Mode {
        STOPPED,
        INTAKE,
        HOLD,
        EJECT
    }

    private final AlgaeClawIO io;
    private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();
    private final PieceSensor sensor = new PieceSensor("Algae sensor offline");
    private Mode mode = Mode.STOPPED;
    private double ejectStartS = Double.NaN;

    public AlgaeClaw(AlgaeClawIO io) {
        this.io = io;
    }

    /** Run inward until an algae is seen, then hold it. */
    public void intake() {
        mode = sensor.present() ? Mode.HOLD : Mode.INTAKE;
    }

    public void hold() {
        mode = Mode.HOLD;
    }

    /** Push the algae out. Keeps running until another mode is chosen. */
    public void eject() {
        if (mode != Mode.EJECT) {
            ejectStartS = Timer.getFPGATimestamp();
        }
        mode = Mode.EJECT;
    }

    public void stop() {
        mode = Mode.STOPPED;
    }

    /** True while an algae is held. False whenever the sensor is offline. */
    public boolean hasAlgae() {
        return sensor.present();
    }

    /** True while the CANrange is answering. */
    public boolean sensorOnline() {
        return sensor.online();
    }

    /**
     * Whether an algae is aboard as far as anyone knows: what the sensor reports while it answers, and
     * the last answer while it does not, until a push out long enough to clear it.
     */
    public boolean mayHaveAlgae() {
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
        Logger.processInputs("AlgaeClaw", inputs);

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

        Logger.recordOutput("AlgaeClaw/Mode", mode);
        Logger.recordOutput("AlgaeClaw/HasAlgae", present);
    }
}
