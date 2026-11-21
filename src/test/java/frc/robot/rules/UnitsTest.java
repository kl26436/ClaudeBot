package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.config.RobotConstants.ElevatorK;
import frc.robot.config.RobotConstants.WristK;
import frc.robot.config.Setpoints;
import org.junit.jupiter.api.Test;

class UnitsTest {
    @Test
    void oneMotorTurnIsOnePoint375Inches() {
        assertEquals(1.375, ElevatorK.IN_PER_MOTOR_ROT, 1e-9);
    }

    @Test
    void fullTravelIs48MotorTurns() {
        assertEquals(48.0, ElevatorK.TRAVEL_IN / ElevatorK.IN_PER_MOTOR_ROT, 1e-9);
    }

    @Test
    void accelCapIsPointSixG() {
        assertEquals(231.65, ElevatorK.MAX_ACCEL_IN_PER_S2, 0.01);
    }

    @Test
    void setpointsAreInsideSoftLimits() {
        for (var p : Setpoints.Pose.values()) {
            assertTrue(p.carriageIn >= 0 && p.carriageIn <= ElevatorK.SOFT_MAX_IN, p.name());
            assertTrue(p.wristDeg >= WristK.MIN_DEG && p.wristDeg <= WristK.MAX_DEG, p.name());
        }
    }
}
