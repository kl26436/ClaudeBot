package frc.robot.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.RobotConstants.DriveK;
import frc.robot.subsystems.drive.DriveToPoint;
import org.junit.jupiter.api.Test;

class DriveToPointTest {
    private final DriveToPoint c = new DriveToPoint(DriveToPoint.KP_AUTO, DriveToPoint.KD);

    @Test void pointsAtTheTarget() { var o = c.calculate(new Translation2d(), new Translation2d(1, 1), 3, 0.02); assertEquals(o.vxMps(), o.vyMps(), 1e-9); assertTrue(o.vxMps() > 0); }
    @Test void capsAtMaxSpeed() { var o = c.calculate(new Translation2d(), new Translation2d(5, 0), 2.0, 0.02); assertEquals(2.0, Math.hypot(o.vxMps(), o.vyMps()), 1e-9); }
    @Test void stopsInsideTolerance() { var o = c.calculate(new Translation2d(), new Translation2d(0.005, 0), 2.0, 0.02); assertEquals(0, o.vxMps(), 1e-12); }

    @Test void firstCallHasNoDerivativeKick() {
        var o = c.calculate(new Translation2d(), new Translation2d(0.1, 0), 3, 0.02);
        assertEquals(DriveToPoint.KP_AUTO * 0.1 + DriveToPoint.FRICTION_FRACTION * DriveK.MAX_SPEED_MPS, o.vxMps(), 1e-9);
        assertEquals(0, o.vyMps(), 1e-9);
    }

    @Test void changingTheTargetResetsTheDerivative() {
        c.calculate(new Translation2d(), new Translation2d(0.1, 0), 3, 0.02);
        var o = c.calculate(new Translation2d(), new Translation2d(0, 0.2), 3, 0.02);
        assertEquals(0, o.vxMps(), 1e-9);
        assertEquals(DriveToPoint.KP_AUTO * 0.2 + DriveToPoint.FRICTION_FRACTION * DriveK.MAX_SPEED_MPS, o.vyMps(), 1e-9);
    }

    @Test void approachingSlowsTheCommand() {
        c.calculate(new Translation2d(), new Translation2d(1, 0), 10, 0.02);
        var o = c.calculate(new Translation2d(0.05, 0), new Translation2d(1, 0), 10, 0.02);
        assertEquals(DriveToPoint.KP_AUTO * 0.95 + DriveToPoint.KD * (0.95 - 1.0) / 0.02
                + DriveToPoint.FRICTION_FRACTION * DriveK.MAX_SPEED_MPS, o.vxMps(), 1e-9);
    }
}
