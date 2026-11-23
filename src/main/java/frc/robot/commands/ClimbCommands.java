package frc.robot.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.Wanted;

/** The climb button. */
public final class ClimbCommands {
    private ClimbCommands() {}

    /** With the field attached, the climb is allowed only this close to the end of teleop. */
    static final double ENDGAME_S = 30; // PLACEHOLDER(tuning)

    private static final Alert LOCKED = new Alert("Climb is locked until the last 30 s", AlertType.kInfo);

    /**
     * Opening the funnel cannot be undone in a match, so a stray press must not do it: only in teleop
     * and, on a field, only in the endgame. A climb already begun may always go on.
     */
    public static boolean allowed(boolean teleop, boolean fieldAttached, double matchTimeS) {
        return teleop && (!fieldAttached || (matchTimeS >= 0 && matchTimeS <= ENDGAME_S));
    }

    private static boolean allowedNow() {
        return allowed(DriverStation.isTeleopEnabled(), DriverStation.isFMSAttached(), DriverStation.getMatchTime());
    }

    /**
     * While held, go through the climb one step at a time (see {@code Climber}). The driver keeps the
     * drivetrain throughout. Letting go before the funnel opens stows; after that it only pauses.
     */
    public static Command climb(RobotContainer r) {
        return new Command() {
            {
                addRequirements(r.climber(), r.superstructure());
                setName("Climb");
            }

            private boolean running;

            @Override
            public void initialize() {
                running = r.climber().committed() || allowedNow();
                LOCKED.set(!running);
                if (running) {
                    r.superstructure().setWanted(Wanted.CLIMB);
                }
            }

            @Override
            public void execute() {
                r.climber().setHeld(running);
            }

            @Override
            public void end(boolean interrupted) {
                LOCKED.set(false);
                r.climber().setHeld(false);
                if (running && !r.climber().committed()) {
                    r.superstructure().setWanted(Wanted.STOW);
                }
            }
        };
    }
}
