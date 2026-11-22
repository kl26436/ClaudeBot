// Adapted from FRC Team 2910's 2025CompetitionRobot-Public (MIT licence; see NOTICE).
package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoreCommands;
import frc.robot.field.Barge;
import frc.robot.field.Reef.Branch;
import frc.robot.field.StartPose;
import frc.robot.field.Station;
import frc.robot.subsystems.Superstructure.Wanted;
import java.util.List;
import java.util.Optional;

/**
 * Builds each auto for one alliance. One package-private method per {@link Auto}, built from the
 * same scoring and pickup commands the driver uses. Every leg has a timeout, so a late or missing
 * coral costs one piece and not the routine.
 */
class AutoFactory {
    static final double SCORE_SPEED_MPS = 3.0; // PLACEHOLDER(tuning)
    static final double STATION_SPEED_MPS = 3.0; // PLACEHOLDER(tuning)
    static final double NET_SPEED_MPS = 2.5; // PLACEHOLDER(tuning)

    /** Longest a station leg waits at the station for a coral. */
    static final double STATION_WAIT_S = 2.0;

    /** How close to the pickup pose counts as at the station, for that wait. */
    static final double AT_STATION_M = 0.15;

    static final double SCORE_LEG_TIMEOUT_S = 6.0; // PLACEHOLDER(tuning)
    static final double STATION_LEG_TIMEOUT_S = 6.0; // PLACEHOLDER(tuning)
    static final double ALGAE_LEG_TIMEOUT_S = 4.0; // PLACEHOLDER(tuning)
    static final double NET_LEG_TIMEOUT_S = 4.0; // PLACEHOLDER(tuning)

    /** How far Leave drives forward off the starting line. */
    static final double LEAVE_DISTANCE_M = 1.0; // PLACEHOLDER(field-check)

    static final double LEAVE_TIMEOUT_S = 3.0;

    private final Alliance alliance;
    private final RobotContainer r;

    AutoFactory(Alliance alliance, RobotContainer r) {
        this.alliance = alliance;
        this.r = r;
    }

    // ---- Legs ----

    /** Score on a branch at L4, or skip straight on if no coral is on board when the leg starts. */
    private Command scoreLeg(Branch b) {
        return Commands.either(
                        ScoreCommands.alignAndScore(r, () -> b, () -> Wanted.L4, SCORE_SPEED_MPS, false)
                                .withTimeout(SCORE_LEG_TIMEOUT_S),
                        Commands.none(),
                        () -> r.superstructure().hasCoral())
                .withName("Score " + b);
    }

    /** Drive to the station and take a coral, waiting there at most {@link #STATION_WAIT_S}. */
    private Command stationLeg(Station.Side side) {
        Pose2d pickup = Station.pickupPose(alliance, side);
        return ScoreCommands.stationPickup(r, () -> side, STATION_SPEED_MPS, false)
                .raceWith(Commands.waitUntil(
                                () -> r.drive().pose().getTranslation().getDistance(pickup.getTranslation()) <= AT_STATION_M)
                        .andThen(Commands.waitSeconds(STATION_WAIT_S)))
                .withTimeout(STATION_LEG_TIMEOUT_S)
                .withName("Station " + side);
    }

    private static Optional<Pose2d> from(StartPose p, Alliance a) {
        return Optional.of(p.pose(a));
    }

    private AutoRoutine threeCoral(StartPose start, Station.Side side, Branch first, Branch second, Branch third) {
        return new AutoRoutine(
                from(start, alliance),
                Commands.sequence(
                        scoreLeg(first), stationLeg(side), scoreLeg(second), stationLeg(side), scoreLeg(third)),
                List.of(first, second, third));
    }

    // ---- One method per Auto ----

    AutoRoutine idle() {
        return new AutoRoutine(Optional.empty(), Commands.none(), List.of());
    }

    AutoRoutine leave() {
        Command drive = ScoreCommands.driveTo(
                        r,
                        a -> r.drive().pose().transformBy(new Transform2d(LEAVE_DISTANCE_M, 0, Rotation2d.kZero)),
                        SCORE_SPEED_MPS)
                .withTimeout(LEAVE_TIMEOUT_S);
        return new AutoRoutine(Optional.empty(), drive, List.of());
    }

    AutoRoutine oneRight() {
        return new AutoRoutine(from(StartPose.RIGHT, alliance), scoreLeg(Branch.E), List.of(Branch.E));
    }

    AutoRoutine oneCentre() {
        return new AutoRoutine(from(StartPose.CENTRE, alliance), scoreLeg(Branch.H), List.of(Branch.H));
    }

    AutoRoutine oneLeft() {
        return new AutoRoutine(from(StartPose.LEFT, alliance), scoreLeg(Branch.J), List.of(Branch.J));
    }

    AutoRoutine threeRightECD() {
        return threeCoral(StartPose.RIGHT, Station.Side.RIGHT, Branch.E, Branch.C, Branch.D);
    }

    AutoRoutine threeLeftJLK() {
        return threeCoral(StartPose.LEFT, Station.Side.LEFT, Branch.J, Branch.L, Branch.K);
    }

    /** H at L4, then that face's algae, then the net. */
    AutoRoutine centreHAlgae() {
        int face = Branch.H.face();
        Command command = Commands.sequence(
                scoreLeg(Branch.H),
                ScoreCommands.algaeFromReef(r, () -> face).withTimeout(ALGAE_LEG_TIMEOUT_S),
                Commands.either(
                        Commands.sequence(
                                ScoreCommands.driveTo(r, Barge::netPose, NET_SPEED_MPS).withTimeout(NET_LEG_TIMEOUT_S),
                                ScoreCommands.netScore(r).withTimeout(NET_LEG_TIMEOUT_S)),
                        Commands.none(),
                        () -> r.superstructure().hasAlgae()));
        return new AutoRoutine(from(StartPose.CENTRE, alliance), command, List.of(Branch.H));
    }
}
