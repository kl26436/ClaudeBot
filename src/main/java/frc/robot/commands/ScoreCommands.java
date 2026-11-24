package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.field.Reef;
import frc.robot.field.Station;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Wanted;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/** The scoring and pickup moves, shared by the driver's buttons and the autos. */
public final class ScoreCommands {
    private ScoreCommands() {}

    /** The elevator starts rising once the robot is this close to its scoring pose. */
    public static final double RAISE_DISTANCE_M = Units.inchesToMeters(48);

    /** After a release request, a coral still seen after this long ends the score anyway. */
    static final double RELEASE_TIMEOUT_S = 1.0;

    /** Keep pushing at least this long after the release starts, even once the sensor reads empty. */
    static final double MIN_RELEASE_S = 0.25; // PLACEHOLDER(tuning)

    /**
     * Whether a release requested at {@code requestedAtS} is over. It must have actually started at the
     * pose; then it ends once an answering sensor reads the piece gone (after {@link #MIN_RELEASE_S}),
     * or after {@link #RELEASE_TIMEOUT_S}, which is the only end when the sensor is offline. A release
     * that never starts also gives up after {@link #RELEASE_TIMEOUT_S}.
     */
    static boolean releaseOver(
            Superstructure s, double requestedAtS, boolean pieceSeen, boolean sensorOnline) {
        double t = Timer.getFPGATimestamp() - requestedAtS;
        if (!s.isReleasing()) {
            return t >= RELEASE_TIMEOUT_S;
        }
        boolean seenGone = sensorOnline && !pieceSeen;
        return (seenGone && t >= MIN_RELEASE_S) || t >= RELEASE_TIMEOUT_S;
    }

    /** A camera that disputes the position at the target holds the release this long, no more. */
    static final double DOUBT_LIMIT_S = 0.6; // PLACEHOLDER(tuning)

    /** An estimate that moves this far in one loop has been corrected: the route is planned again from there. */
    static final double REPLAN_JUMP_M = 0.4;

    /** A route's via point counts as passed this close to it. */
    static final double VIA_PASSED_M = 0.25;

    /** Speed cap for the algae pickup drive. */
    static final double ALGAE_SPEED_MPS = 2.0; // PLACEHOLDER(tuning)

    /**
     * Drive {@link Reef#route} to the branch, raise to the level within {@link #RAISE_DISTANCE_M},
     * and release once the drive and the mechanism are both on target and the camera does not dispute
     * where that is (for at most {@link #DOUBT_LIMIT_S}). Finishes when the coral is
     * gone or {@link #RELEASE_TIMEOUT_S} after the release. On end or interrupt: clear the release
     * and the vision face, and stow. Ends at once if the alliance is unknown.
     */
    public static Command alignAndScore(
            RobotContainer r,
            Supplier<Reef.Branch> branch,
            Supplier<Wanted> level,
            double maxSpeedMps,
            boolean teleopGains) {
        return new RouteCommand(r, a -> Reef.scoringPose(a, branch.get()), maxSpeedMps, teleopGains) {
            private Wanted chosen;
            private double releasedAtS;
            private double readySinceS;

            @Override
            void start(Alliance a, Pose2d target) {
                chosen = level.get();
                releasedAtS = Double.NaN;
                readySinceS = Double.NaN;
                r.vision().setTargetFace(Optional.of(Reef.nearestFace(a, target.getTranslation())));
            }

            @Override
            void step() {
                var s = r.superstructure();
                if (distanceToTargetM() <= RAISE_DISTANCE_M && s.wanted() != chosen) {
                    s.setWanted(chosen);
                }
                double now = Timer.getFPGATimestamp();
                if (!(onFinalLeg() && r.drive().atTarget() && s.atPose())) {
                    readySinceS = Double.NaN;
                } else if (Double.isNaN(releasedAtS)) {
                    if (Double.isNaN(readySinceS)) {
                        readySinceS = now;
                    }
                    if (!r.vision().doubtsThePosition() || now - readySinceS >= DOUBT_LIMIT_S) {
                        s.requestRelease(true);
                        releasedAtS = now;
                    }
                }
            }

            @Override
            boolean done() {
                var s = r.superstructure();
                return !Double.isNaN(releasedAtS)
                        && releaseOver(s, releasedAtS, s.hasCoral(), s.coralSensorOnline());
            }

            @Override
            void finish() {
                r.superstructure().requestRelease(false);
                r.vision().setTargetFace(Optional.empty());
                r.superstructure().setWanted(Wanted.STOW);
            }
        };
    }

    /** Drive to the station's pickup pose with the coral intake running; finish when a coral is seen. */
    public static Command stationPickup(
            RobotContainer r, Supplier<Station.Side> side, double maxSpeedMps, boolean teleopGains) {
        return new RouteCommand(r, a -> Station.pickupPose(a, side.get()), maxSpeedMps, teleopGains) {
            @Override
            void start(Alliance a, Pose2d target) {
                r.superstructure().setWanted(Wanted.STATION_INTAKE);
            }

            @Override
            boolean done() {
                return r.superstructure().hasCoral();
            }

            @Override
            void finish() {
                r.superstructure().setWanted(Wanted.STOW);
            }
        };
    }

    /**
     * Drive to a face's algae pose with the claw at that face's algae height; finish when the algae is
     * held. Stowing with an algae on board carries it.
     */
    public static Command algaeFromReef(RobotContainer r, IntSupplier face) {
        return new RouteCommand(r, a -> Reef.algaePose(a, face.getAsInt()), ALGAE_SPEED_MPS, false) {
            private Wanted height;

            @Override
            void start(Alliance a, Pose2d target) {
                int f = Reef.nearestFace(a, target.getTranslation());
                height = Reef.algaeIsHigh(f) ? Wanted.ALGAE_HIGH : Wanted.ALGAE_LOW;
                r.vision().setTargetFace(Optional.of(f));
            }

            @Override
            void step() {
                if (distanceToTargetM() <= RAISE_DISTANCE_M && r.superstructure().wanted() != height) {
                    r.superstructure().setWanted(height);
                }
            }

            @Override
            boolean done() {
                return r.superstructure().hasAlgae();
            }

            @Override
            void finish() {
                r.vision().setTargetFace(Optional.empty());
                r.superstructure().setWanted(Wanted.STOW);
            }
        };
    }

    /** Drive to a pose (around the reef if need be), the mechanism left as it is; finish on target. */
    public static Command driveTo(RobotContainer r, Function<Alliance, Pose2d> target, double maxSpeedMps) {
        return new RouteCommand(r, target, maxSpeedMps, false) {
            @Override
            boolean done() {
                return onFinalLeg() && r.drive().atTarget();
            }

            @Override
            void finish() {}
        };
    }

    /** Raise to the net and release the algae; the driver (or the auto) has already driven there. */
    public static Command netScore(RobotContainer r) {
        return releaseAlgaeAt(r, Wanted.NET);
    }

    /** Provisional: the processor pose and release, as for the net. */
    public static Command processorScore(RobotContainer r) {
        return releaseAlgaeAt(r, Wanted.PROCESSOR);
    }

    private static Command releaseAlgaeAt(RobotContainer r, Wanted pose) {
        Superstructure s = r.superstructure();
        double[] releasedAt = {Double.NaN};
        // Requires the superstructure, so Back (stow and cancel) interrupts it.
        return Commands.sequence(
                        Commands.runOnce(() -> s.setWanted(pose), s),
                        Commands.waitUntil(s::atPose),
                        Commands.runOnce(() -> {
                            s.requestRelease(true);
                            releasedAt[0] = Timer.getFPGATimestamp();
                        }),
                        Commands.waitUntil(() -> releaseOver(s, releasedAt[0], s.hasAlgae(), s.algaeSensorOnline())))
                .finallyDo(() -> {
                    s.requestRelease(false);
                    s.setWanted(Wanted.STOW);
                })
                .withName(pose + " release");
    }

    /**
     * Drives a route planned once at the start (around the reef when the straight line would cross
     * it) and lets the subclass run the mechanism along the way. Requires the drive and the
     * superstructure. Ends at once, doing nothing, when the alliance is unknown.
     */
    private abstract static class RouteCommand extends Command {
        private final RobotContainer r;
        private final Function<Alliance, Pose2d> targetFor;
        private final double maxSpeedMps;
        private final boolean teleopGains;
        private List<Pose2d> route = List.of();
        private int leg;
        private Pose2d target;
        private Alliance alliance;
        private Translation2d lastAt;
        private boolean noAlliance;

        RouteCommand(RobotContainer r, Function<Alliance, Pose2d> targetFor, double maxSpeedMps, boolean teleopGains) {
            this.r = r;
            this.targetFor = targetFor;
            this.maxSpeedMps = maxSpeedMps;
            this.teleopGains = teleopGains;
            addRequirements(r.drive(), r.superstructure());
        }

        void start(Alliance a, Pose2d target) {}

        void step() {}

        abstract boolean done();

        abstract void finish();

        final double distanceToTargetM() {
            return r.drive().pose().getTranslation().getDistance(target.getTranslation());
        }

        final boolean onFinalLeg() {
            return leg == route.size() - 1;
        }

        @Override
        public final void initialize() {
            Optional<Alliance> known = RobotState.get().alliance();
            noAlliance = known.isEmpty();
            if (noAlliance) {
                return;
            }
            alliance = known.get();
            target = targetFor.apply(alliance);
            start(alliance, target);
            plan();
        }

        private void plan() {
            lastAt = r.drive().pose().getTranslation();
            route = Reef.route(alliance, lastAt, target);
            leg = 0;
            driveLeg();
        }

        private void driveLeg() {
            r.drive().driveToPoint(route.get(leg), maxSpeedMps, teleopGains);
        }

        @Override
        public final void execute() {
            if (noAlliance) {
                return;
            }
            SwerveSubsystem drive = r.drive();
            Translation2d at = drive.pose().getTranslation();
            if (at.getDistance(lastAt) > REPLAN_JUMP_M) {
                plan(); // the camera has moved the estimate: the old route started somewhere else
            }
            lastAt = at;
            if (!onFinalLeg()
                    && drive.pose().getTranslation().getDistance(route.get(leg).getTranslation()) <= VIA_PASSED_M) {
                leg++;
                driveLeg();
            }
            step();
        }

        @Override
        public final boolean isFinished() {
            return noAlliance || done();
        }

        @Override
        public final void end(boolean interrupted) {
            r.drive().stopDriving();
            if (!noAlliance) {
                finish();
            }
        }
    }
}
