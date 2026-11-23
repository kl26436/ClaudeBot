package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.Wanted;

/** The climb buttons. */
public final class ClimbCommands {
    private ClimbCommands() {}

    /** With the field attached, the climb is allowed only this close to the end of teleop. */
    static final double ENDGAME_S = 30; // PLACEHOLDER(tuning)

    /**
     * Opening the funnel cannot be undone in a match, so a stray press must not do it: only in teleop
     * and, on a field, only in the endgame.
     */
    public static boolean allowed(boolean teleop, boolean fieldAttached, double matchTimeS) {
        return teleop && (!fieldAttached || (matchTimeS >= 0 && matchTimeS <= ENDGAME_S));
    }

    /** Whether the climb button does anything now. A climb already begun may always go on. */
    public static boolean available(RobotContainer r) {
        return r.climber().committed()
                || allowed(DriverStation.isTeleopEnabled(), DriverStation.isFMSAttached(), DriverStation.getMatchTime());
    }

    /**
     * While held, go through the climb one step at a time (see {@code Climber}). The driver keeps the
     * drivetrain throughout. Letting go before the funnel opens stows; after that it only pauses.
     */
    public static Command climb(RobotContainer r) {
        return Commands.startRun(
                        () -> r.superstructure().setWanted(Wanted.CLIMB),
                        () -> r.climber().setHeld(true),
                        r.climber(),
                        r.superstructure())
                .finallyDo(() -> {
                    r.climber().setHeld(false);
                    if (!r.climber().committed()) {
                        r.superstructure().setWanted(Wanted.STOW);
                    }
                })
                .withName("Climb");
    }

    /** While held, let an empty fork back down to level, for another try at the cage. */
    public static Command lowerFork(RobotContainer r) {
        return Commands.run(() -> r.climber().setLowerHeld(true), r.climber())
                .finallyDo(() -> r.climber().setLowerHeld(false))
                .withName("Lower fork");
    }

    /** For the pit, disabled, once the fork is stowed and the tray latched by hand. */
    public static Command pitReset(RobotContainer r) {
        return Commands.runOnce(r.climber()::pitReset).ignoringDisable(true).withName("Pit reset");
    }
}
