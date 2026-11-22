package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.Reef;
import java.util.List;
import java.util.Optional;

/**
 * One auto built for one alliance: where it starts (empty for "anywhere"), what it runs, and the
 * branches it should score on, in order.
 */
public record AutoRoutine(Optional<Pose2d> startPose, Command command, List<Reef.Branch> expectedCoral) {}
