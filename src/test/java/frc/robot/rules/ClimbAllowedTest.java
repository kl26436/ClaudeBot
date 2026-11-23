package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.commands.ClimbCommands;
import org.junit.jupiter.api.Test;

/** Opening the funnel is one shot per match, so a stray press must not start a climb. */
class ClimbAllowedTest {
    @Test void neverOutsideTeleop() {
        assertFalse(ClimbCommands.allowed(false, true, 10));
        assertFalse(ClimbCommands.allowed(false, false, -1));
    }

    @Test void onAFieldOnlyInTheEndgame() {
        assertFalse(ClimbCommands.allowed(true, true, 135));
        assertFalse(ClimbCommands.allowed(true, true, 31));
        assertTrue(ClimbCommands.allowed(true, true, 30));
        assertTrue(ClimbCommands.allowed(true, true, 0));
        assertFalse(ClimbCommands.allowed(true, true, -1), "no match clock yet");
    }

    @Test void anyTimeInPractice() {
        assertTrue(ClimbCommands.allowed(true, false, -1));
        assertTrue(ClimbCommands.allowed(true, false, 120));
    }
}
