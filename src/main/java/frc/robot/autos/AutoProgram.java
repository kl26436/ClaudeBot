// Adapted from FRC Team 2910's 2025CompetitionRobot-Public (MIT licence; see NOTICE).
package frc.robot.autos;

import java.util.function.Function;

/** An auto, its dashboard label, and how to build it from an alliance's {@link AutoFactory}. */
public record AutoProgram(Auto auto, String label, Function<AutoFactory, AutoRoutine> build) {}
