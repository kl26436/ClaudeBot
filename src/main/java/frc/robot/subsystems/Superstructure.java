package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConstants.DriveK;
import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.Setpoints;
import frc.robot.config.Setpoints.Pose;
import frc.robot.subsystems.SuperstructurePlanner.Targets;
import frc.robot.subsystems.claw.AlgaeClaw;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates the elevator, wrist, coral rollers and algae claw. Each loop it maps the wanted state
 * to a pose and lets {@link SuperstructurePlanner} pick targets for both axes that keep every move
 * inside the keep-out table.
 *
 * <p>An axis whose sensor cannot be trusted freezes the other one: an untrusted wrist holds the
 * carriage where it is (so a robot that boots with a bad wrist encoder never leaves the bottom), and
 * an un-homed elevator keeps the wrist where it is.
 */
public class Superstructure extends SubsystemBase {
    public enum Wanted {
        STOW,
        STATION_INTAKE,
        L1,
        L2,
        L3,
        L4,
        ALGAE_LOW,
        ALGAE_HIGH,
        NET,
        PROCESSOR,
        /** Out of the climber's way: wrist level, carriage just off the bottom. */
        CLIMB
    }

    /** Drive speed is full up to this carriage height, then falls linearly to {@link #MIN_SPEED_SCALE} at the top. */
    static final double FULL_SPEED_MAX_IN = 23.0; // PLACEHOLDER(tuning)

    static final double MIN_SPEED_SCALE = 0.35; // PLACEHOLDER(tuning)

    /** Fraction of the tipping acceleration the drive may use. */
    static final double TIP_SAFETY = 0.8; // PLACEHOLDER(tuning)

    /** How far the wrist and carriage may be from the climb pose with the fork moving. */
    static final double CLIMB_CLEAR_DEG = 8;

    static final double CLIMB_CLEAR_IN = 1.0;

    /** An algae held when the climb starts is dropped: pushed out with the carriage below this. */
    static final double CLIMB_DROP_MAX_IN = 5;

    /** After pushing this long the algae is taken to be gone, whatever its sensor says. */
    static final double CLIMB_DROP_S = 1.0;

    private static final double STANDARD_G = 9.80665;

    private final Elevator elevator;
    private final Wrist wrist;
    private final CoralRollers coral;
    private final AlgaeClaw claw;

    // Read by the simulation thread in simulateJava.
    private volatile Wanted wanted = Wanted.STOW;
    private boolean releaseRequested = false;
    /** Set once a requested release has started; it then runs until the request is cleared. */
    private boolean releasing = false;
    /** Carriage height frozen while the wrist is untrusted, or NaN. */
    private double heldCarriageIn = Double.NaN;
    /** Wrist angle frozen while the elevator is un-homed, or NaN. */
    private double heldWristDeg = Double.NaN;
    private Targets target = new Targets(0, Pose.STOW.wristDeg);
    /** True once the climber is committed: the mechanism stays in the climb pose. */
    private BooleanSupplier climbLock = () -> false;

    private double climbDropStartS = Double.NaN;

    public Superstructure(Elevator elevator, Wrist wrist, CoralRollers coral, AlgaeClaw claw) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.coral = coral;
        this.claw = claw;
    }

    /** Hold the mechanism in {@link Wanted#CLIMB}, whatever else is asked for, while {@code lock} is true. */
    public void setClimbLock(BooleanSupplier lock) {
        climbLock = lock;
    }

    /**
     * Choose what the mechanism should do. Changing it clears a pending release. Ignored once the
     * climber is committed.
     */
    public void setWanted(Wanted w) {
        if (climbLock.getAsBoolean()) {
            w = Wanted.CLIMB;
        }
        if (w != wanted) {
            releaseRequested = false;
            releasing = false;
        }
        wanted = w;
    }

    public Wanted wanted() {
        return wanted;
    }

    /**
     * Ask to let go of the piece the current pose is for. The release starts only once {@link
     * #atPose()} is true, then runs until the request is cleared or the wanted state changes, so a
     * wobble mid-release cannot leave a coral half out.
     */
    public void requestRelease(boolean release) {
        releaseRequested = release;
        if (!release) {
            releasing = false;
        }
    }

    /** The pose the wanted state means right now: STOW while holding an algae is the algae carry. */
    public Pose pose() {
        return switch (wanted) {
            case STOW -> claw.mayHaveAlgae() ? Pose.ALGAE_CARRY : Pose.STOW;
            case STATION_INTAKE -> Pose.STATION;
            case L1 -> Pose.L1;
            case L2 -> Pose.L2;
            case L3 -> Pose.L3;
            case L4 -> Pose.L4;
            case ALGAE_LOW -> Pose.ALGAE_LOW;
            case ALGAE_HIGH -> Pose.ALGAE_HIGH;
            case NET -> Pose.NET;
            case PROCESSOR -> Pose.PROCESSOR;
            case CLIMB -> Pose.CLIMB;
        };
    }

    /** True when both axes are trusted and within tolerance of the wanted pose itself, not a waypoint. */
    public boolean atPose() {
        if (!elevator.isHomed() || !wristUsable()) {
            return false;
        }
        Pose p = pose();
        return Math.abs(elevator.carriageIn() - p.carriageIn) <= Setpoints.CARRIAGE_TOL_IN
                && Math.abs(wrist.wristDeg() - p.wristDeg) <= Setpoints.WRIST_TOL_DEG;
    }

    /** True in the climb pose itself: the funnel may be opened. */
    public boolean atClimbPose() {
        return wanted == Wanted.CLIMB && atPose();
    }

    /** True while the mechanism is close enough to the climb pose for the fork to move. */
    public boolean clearOfClimber() {
        return wanted == Wanted.CLIMB
                && elevator.isHomed()
                && wristUsable()
                && Math.abs(elevator.carriageIn() - Pose.CLIMB.carriageIn) <= CLIMB_CLEAR_IN
                && Math.abs(wrist.wristDeg() - Pose.CLIMB.wristDeg) <= CLIMB_CLEAR_DEG;
    }

    /** True once a requested release has actually started at the pose (the rollers are pushing). */
    public boolean isReleasing() {
        return releasing;
    }

    /** A coral seen by an answering sensor. False while the coral sensor is offline. */
    public boolean hasCoral() {
        return coral.hasCoral();
    }

    /** An algae seen by an answering sensor. False while the algae sensor is offline. */
    public boolean hasAlgae() {
        return claw.hasAlgae();
    }

    public boolean coralSensorOnline() {
        return coral.sensorOnline();
    }

    public boolean algaeSensorOnline() {
        return claw.sensorOnline();
    }

    /** Fraction of full drive speed allowed at the present carriage height. */
    public double driveSpeedScale() {
        double h = elevator.carriageIn();
        if (!Double.isFinite(h)) {
            return MIN_SPEED_SCALE;
        }
        double t = MathUtil.clamp((h - FULL_SPEED_MAX_IN) / (ElevatorK.SOFT_MAX_IN - FULL_SPEED_MAX_IN), 0, 1);
        return 1.0 - (1.0 - MIN_SPEED_SCALE) * t;
    }

    /**
     * Drive acceleration allowed at the present carriage height: {@link #TIP_SAFETY} of the acceleration
     * that would tip the robot there (drawing R-001), interpolated between the drawing's heights. An
     * unknown height is treated as full height.
     */
    public double driveAccelLimitMps2() {
        double h = elevator.carriageIn();
        if (!Double.isFinite(h)) {
            h = ElevatorK.TRAVEL_IN;
        }
        double[] xs = DriveK.TIP_CARRIAGE_IN;
        double[] gs = DriveK.TIP_ACCEL_G;
        double tipG = gs[gs.length - 1];
        if (h <= xs[0]) {
            tipG = gs[0];
        } else {
            for (int i = 0; i + 1 < xs.length; i++) {
                if (h <= xs[i + 1]) {
                    tipG = gs[i] + (gs[i + 1] - gs[i]) * (h - xs[i]) / (xs[i + 1] - xs[i]);
                    break;
                }
            }
        }
        return TIP_SAFETY * tipG * STANDARD_G;
    }

    public Command setWantedCommand(Wanted w) {
        return runOnce(() -> setWanted(w));
    }

    /** Nothing to release yet; kept so callers can close every subsystem the same way. */
    public void close() {}

    private boolean wristUsable() {
        return wrist.isTrusted() && Double.isFinite(wrist.wristDeg());
    }

    @Override
    public void periodic() {
        if (climbLock.getAsBoolean()) {
            setWanted(Wanted.CLIMB);
        }
        if (wanted != Wanted.CLIMB) {
            climbDropStartS = Double.NaN;
        }
        Pose p = pose();
        moveAxes(p);
        runRollers();

        Logger.recordOutput("Superstructure/Wanted", wanted);
        Logger.recordOutput("Superstructure/Pose", p);
        Logger.recordOutput("Superstructure/AtPose", atPose());
        Logger.recordOutput("Superstructure/TargetCarriageIn", target.carriageIn());
        Logger.recordOutput("Superstructure/TargetWristDeg", target.wristDeg());
        Logger.recordOutput("Superstructure/Releasing", releasing);
        Logger.recordOutput("Superstructure/DriveSpeedScale", driveSpeedScale());
    }

    private void moveAxes(Pose p) {
        boolean homed = elevator.isHomed();
        boolean wristOk = wristUsable();

        if (!wristOk) {
            // Wrist angle unknown: freeze the carriage where it is. It never rises, so from the
            // bottom it stays under UNTRUSTED_WRIST_MAX_IN; in the air, driving down could swing an
            // unseen wrist through the funnel, so it stays up.
            if (Double.isNaN(heldCarriageIn)) {
                double h = elevator.carriageIn();
                heldCarriageIn = Double.isFinite(h) ? Math.max(0, h) : 0;
            }
            elevator.setGoalIn(heldCarriageIn);
            target = new Targets(heldCarriageIn, wrist.wristDeg());
            return;
        }
        heldCarriageIn = Double.NaN;

        if (!homed) {
            // Carriage height unknown: the wrist stays where it is until the elevator has homed.
            if (Double.isNaN(heldWristDeg)) {
                heldWristDeg = wrist.wristDeg();
            }
            wrist.setGoalDeg(heldWristDeg);
            target = new Targets(elevator.carriageIn(), heldWristDeg);
            return;
        }
        heldWristDeg = Double.NaN;

        target = SuperstructurePlanner.next(
                elevator.carriageIn(),
                elevator.velocityInPerS(),
                wrist.wristDeg(),
                wrist.velocityDegPerS(),
                new Targets(p.carriageIn, p.wristDeg),
                claw.mayHaveAlgae() && !climbDropDone());
        elevator.setGoalIn(target.carriageIn());
        wrist.setGoalDeg(target.wristDeg());
    }

    private boolean climbDropDone() {
        return !Double.isNaN(climbDropStartS) && Timer.getFPGATimestamp() - climbDropStartS >= CLIMB_DROP_S;
    }

    /** The climb pose is below the angle an algae can be carried at, so it has to go first. */
    private boolean droppingAlgaeToClimb() {
        if (wanted != Wanted.CLIMB || !claw.mayHaveAlgae() || climbDropDone()) {
            return false;
        }
        if (!elevator.isHomed() || elevator.carriageIn() > CLIMB_DROP_MAX_IN) {
            return false;
        }
        if (Double.isNaN(climbDropStartS)) {
            climbDropStartS = Timer.getFPGATimestamp();
        }
        return true;
    }

    private void runRollers() {
        if (releaseRequested && !releasing && atPose()) {
            releasing = true;
        }
        boolean coralRelease = releasing && releasesCoral(wanted);
        boolean algaeRelease = (releasing && releasesAlgae(wanted)) || droppingAlgaeToClimb();

        if (coralRelease) {
            coral.eject();
        } else if (wanted == Wanted.STATION_INTAKE) {
            coral.intake();
        } else if (coral.mayHaveCoral()) {
            coral.hold();
        } else {
            coral.stop();
        }

        if (algaeRelease) {
            claw.eject();
        } else if (wanted == Wanted.ALGAE_LOW || wanted == Wanted.ALGAE_HIGH) {
            claw.intake();
        } else if (claw.mayHaveAlgae()) {
            claw.hold();
        } else {
            claw.stop();
        }
    }

    private static boolean releasesCoral(Wanted w) {
        return w == Wanted.L1 || w == Wanted.L2 || w == Wanted.L3 || w == Wanted.L4;
    }

    private static boolean releasesAlgae(Wanted w) {
        return w == Wanted.NET || w == Wanted.PROCESSOR;
    }
}
