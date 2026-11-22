// Adapted from FRC Team 2910's 2025CompetitionRobot-Public (MIT licence; see NOTICE).
package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The dashboard's auto list and "Set pose to auto start" button, under {@code SmartDashboard/Auto}.
 *
 * <p>To add an auto: add a value to {@link Auto}, a method to {@link AutoFactory}, and one row to
 * {@link #PROGRAMS}.
 */
public class AutoChooser {
    /** Labels are the "Name on the dashboard" column of the design. */
    public static final List<AutoProgram> PROGRAMS = List.of(
            new AutoProgram(Auto.IDLE, "Idle", AutoFactory::idle),
            new AutoProgram(Auto.LEAVE, "Leave", AutoFactory::leave),
            new AutoProgram(Auto.ONE_RIGHT, "1 Right", AutoFactory::oneRight),
            new AutoProgram(Auto.ONE_CENTRE, "1 Centre", AutoFactory::oneCentre),
            new AutoProgram(Auto.ONE_LEFT, "1 Left", AutoFactory::oneLeft),
            new AutoProgram(Auto.THREE_RIGHT_ECD, "3 Right: E, C, D", AutoFactory::threeRightECD),
            new AutoProgram(Auto.THREE_LEFT_JLK, "3 Left: J, L, K", AutoFactory::threeLeftJLK),
            new AutoProgram(Auto.CENTRE_H_ALGAE, "Centre: H and algae", AutoFactory::centreHAlgae));

    private final RobotContainer r;
    private final SendableChooser<Auto> chooser = new SendableChooser<>();
    private final Map<Alliance, AutoFactory> factories = new EnumMap<>(Alliance.class);
    private final Map<Alliance, Map<Auto, AutoRoutine>> built = new EnumMap<>(Alliance.class);
    private final Alert noAllianceAlert = new Alert("No alliance: auto not run", AlertType.kError);

    /** Set only by tests, to pick an auto without the dashboard. */
    private Auto override = null;

    /** The selection and alliance the start-pose button was last pressed for. */
    private Auto poseSetFor = null;

    private Optional<Alliance> poseSetAlliance = Optional.empty();

    public AutoChooser(RobotContainer r) {
        this.r = r;
        for (Alliance a : Alliance.values()) {
            factories.put(a, new AutoFactory(a, r));
            built.put(a, new EnumMap<>(Auto.class));
        }
        for (AutoProgram p : PROGRAMS) {
            if (p.auto() == Auto.IDLE) {
                chooser.setDefaultOption(p.label(), p.auto());
            } else {
                chooser.addOption(p.label(), p.auto());
            }
        }
        SmartDashboard.putData("Auto/Chooser", chooser);
        SmartDashboard.putData(
                "Auto/Set pose to auto start",
                Commands.runOnce(this::setPoseToStart).ignoringDisable(true).withName("Set pose to auto start"));
    }

    public Auto selected() {
        if (override != null) {
            return override;
        }
        Auto a = chooser.getSelected();
        return a == null ? Auto.IDLE : a;
    }

    /** Pick an auto as if from the dashboard. For tests. */
    public void select(Auto a) {
        override = a;
    }

    /** Call while disabled: builds the selected auto for both alliances, so a late alliance change is ready. */
    public void update() {
        Auto a = selected();
        for (Alliance alliance : Alliance.values()) {
            routine(alliance, a);
        }
        Logger.recordOutput("Auto/Selected", a.name());
        Logger.recordOutput("Auto/PoseSet", poseSetForSelected());
        SmartDashboard.putBoolean("Auto/PoseSet", poseSetForSelected());
    }

    /** The selected auto built for an alliance. */
    public AutoRoutine routine(Alliance alliance) {
        return routine(alliance, selected());
    }

    /** An auto built for an alliance, built once and kept. */
    public AutoRoutine routine(Alliance alliance, Auto auto) {
        return built.get(alliance).computeIfAbsent(auto, a -> program(a).build().apply(factories.get(alliance)));
    }

    private static AutoProgram program(Auto a) {
        return PROGRAMS.stream().filter(p -> p.auto() == a).findFirst().orElseThrow();
    }

    /** The selected auto's command for the present alliance; empty while the alliance is unknown. */
    public Optional<Command> selectedCommand() {
        return RobotState.get().alliance().map(a -> routine(a).command());
    }

    /** Where the selected auto starts for the present alliance; empty for "anywhere" or no alliance. */
    public Optional<Pose2d> startPose() {
        return RobotState.get().alliance().flatMap(a -> routine(a).startPose());
    }

    /** The dashboard button: put the pose estimate on the selected auto's start pose. */
    public void setPoseToStart() {
        Optional<Alliance> alliance = RobotState.get().alliance();
        if (alliance.isEmpty()) {
            return;
        }
        routine(alliance.get()).startPose().ifPresent(p -> r.drive().resetPose(p));
        poseSetFor = selected();
        poseSetAlliance = alliance;
    }

    /** True once the button has been pressed for this selection and alliance; false after either changes. */
    public boolean poseSetForSelected() {
        Optional<Alliance> alliance = RobotState.get().alliance();
        return alliance.isPresent() && poseSetFor == selected() && poseSetAlliance.equals(alliance);
    }

    /** Called from autonomousInit: the command to run, or empty and an alert if the alliance is unknown. */
    public Optional<Command> startAuto() {
        Optional<Command> command = selectedCommand();
        noAllianceAlert.set(command.isEmpty());
        command.ifPresent(c -> Logger.recordOutput("Auto/Running", selected().name()));
        return command;
    }
}
