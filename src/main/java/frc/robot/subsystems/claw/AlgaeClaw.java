package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PieceSensor;
import org.littletonrobotics.junction.Logger;

/** Algae claw. {@link #intake()} runs inward and switches itself to {@link #hold()} once an algae is seen. */
public class AlgaeClaw extends SubsystemBase {
    static final double INTAKE_VOLTS = 6; // PLACEHOLDER(tuning)
    static final double HOLD_VOLTS = 1.5; // PLACEHOLDER(tuning)
    static final double EJECT_VOLTS = -8; // PLACEHOLDER(tuning)

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
        mode = Mode.EJECT;
    }

    public void stop() {
        mode = Mode.STOPPED;
    }

    /** True while an algae is held. False whenever the sensor is offline. */
    public boolean hasAlgae() {
        return sensor.present();
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
