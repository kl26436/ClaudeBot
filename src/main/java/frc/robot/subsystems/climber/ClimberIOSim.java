package frc.robot.subsystems.climber;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.config.Ports;
import frc.robot.config.RobotConstants.ClimberK;

/**
 * Climber on simulated Phoenix devices. The winch turns the fork through a rope, which only pulls: the
 * fork hangs on it, down to the level stop. With the pawls down the spool cannot pay out past the last
 * ratchet tooth, and a pawl that carries load does not lift, as on the robot.
 */
public class ClimberIOSim extends ClimberIOTalonFX {
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(2);
    private static final double LBIN_TO_NM = 0.1129848;

    /** Reflected motor inertia and the fork, at the fork; and with the robot hanging on the cage. */
    private static final double FORK_KGM2 = 3.2;

    private static final double HANGING_KGM2 = 12;

    /** The fork's own pull on the rope, deploy springs included. */
    private static final double FORK_LBIN = 40;

    private static final double PAWL_MOVE_S = 0.15;
    private static final double TRAY_OPEN_S = 0.5;

    /** A pawl this far clear of its tooth carries no load and can lift. */
    private static final double PAWL_FREE_DEG = 0.3;

    private static final double STEP_S = 0.001;

    private final TalonFXSimState leftSim;
    private final TalonFXSimState rightSim;

    private double ropeDeg = ClimberK.STOWED_DEG;
    private double speedDegPerS = 0;
    /** The ratchet tooth the pawls would hold at. */
    private double toothDeg = ClimberK.STOWED_DEG;

    private boolean cageCaptured = false;
    private boolean liftCommanded = false;
    private boolean pawlsUp = false;
    private double pawlTimerS = 0;
    private boolean funnelOpened = false;
    private double trayDeg = 0;

    public ClimberIOSim() {
        super(Ports.CLIMBER_LEFT, Ports.CLIMBER_RIGHT, Ports.CLIMBER_PAWL_PWM, Ports.FUNNEL_LATCH_PWM);
        leftSim = left.getSimState();
        rightSim = right.getSimState();
        write();
    }

    /** Advance the physics by {@code dt} seconds using the voltage the left motor is commanding. */
    public void simulate(double dt) {
        double battery = RobotController.getBatteryVoltage();
        leftSim.setSupplyVoltage(battery);
        rightSim.setSupplyVoltage(battery);
        double volts = leftSim.getMotorVoltage();
        for (double t = dt; t > 1e-9; t -= STEP_S) {
            step(volts, Math.min(t, STEP_S));
        }
        if (funnelOpened) {
            trayDeg = Math.min(90, trayDeg + 90 * dt / TRAY_OPEN_S);
        }
        write();
    }

    private void step(double volts, double dt) {
        movePawls(dt);
        // Brake mode shorts the windings, so zero volts is the right model when neutral too.
        double motorRadPerS = Math.toRadians(speedDegPerS) * ClimberK.GEAR_RATIO;
        double nm = GEARBOX.getTorque(GEARBOX.getCurrent(motorRadPerS, volts)) * ClimberK.GEAR_RATIO;
        double inertia = FORK_KGM2;
        if (cageCaptured) {
            nm -= ClimberK.CLIMB_LBIN * LBIN_TO_NM;
            inertia = HANGING_KGM2;
        } else if (ropeDeg > ClimberK.LEVEL_STOP_DEG) {
            nm -= FORK_LBIN * LBIN_TO_NM;
        }
        speedDegPerS += Math.toDegrees(nm / inertia) * dt;
        ropeDeg += speedDegPerS * dt;
        if (ropeDeg >= ClimberK.HARD_MAX_DEG) {
            ropeDeg = ClimberK.HARD_MAX_DEG;
            speedDegPerS = Math.min(0, speedDegPerS);
        }
        double tooth = ClimberK.TOOTH_DEG * Math.floor(ropeDeg / ClimberK.TOOTH_DEG + 1e-9);
        if (pawlsUp) {
            toothDeg = tooth;
            return;
        }
        toothDeg = Math.max(toothDeg, tooth);
        if (ropeDeg <= toothDeg) {
            ropeDeg = toothDeg;
            speedDegPerS = Math.max(0, speedDegPerS);
        }
    }

    private void movePawls(double dt) {
        boolean loaded = ropeDeg - toothDeg < PAWL_FREE_DEG;
        if (liftCommanded == pawlsUp || (liftCommanded && loaded)) {
            pawlTimerS = 0; // nothing to do, or the servo pulls on a loaded pawl and nothing happens
            return;
        }
        pawlTimerS += dt;
        if (pawlTimerS >= PAWL_MOVE_S) {
            pawlsUp = liftCommanded;
            pawlTimerS = 0;
        }
    }

    /** Hang the robot on the fork, or let it go. */
    public void setCageCaptured(boolean captured) {
        cageCaptured = captured;
    }

    /** Put the winch back as at the start of a match: stowed, tray closed, pawls down, at rest. */
    public void reset() {
        ropeDeg = ClimberK.STOWED_DEG;
        toothDeg = ClimberK.STOWED_DEG;
        speedDegPerS = 0;
        cageCaptured = false;
        pawlsUp = false;
        liftCommanded = false;
        pawlTimerS = 0;
        funnelOpened = false;
        trayDeg = 0;
        write();
    }

    /** Where the fork really is: on the rope, or on the level stop with the rope slack. */
    public double trueForkDeg() {
        return Math.max(ropeDeg, ClimberK.LEVEL_STOP_DEG);
    }

    /** How far the funnel tray has swung open, 0 to 90. */
    public double trayDeg() {
        return trayDeg;
    }

    public boolean pawlsUp() {
        return pawlsUp;
    }

    @Override
    public void setPawlsLifted(boolean lifted) {
        super.setPawlsLifted(lifted);
        liftCommanded = lifted;
    }

    @Override
    public void setFunnelOpen(boolean open) {
        super.setFunnelOpen(open);
        funnelOpened |= open;
    }

    private void write() {
        double rotations = (ropeDeg - ClimberK.STOWED_DEG) / 360 * ClimberK.GEAR_RATIO;
        double rps = speedDegPerS / 360 * ClimberK.GEAR_RATIO;
        leftSim.setRawRotorPosition(rotations);
        leftSim.setRotorVelocity(rps);
        rightSim.setRawRotorPosition(-rotations);
        rightSim.setRotorVelocity(-rps);
    }
}
