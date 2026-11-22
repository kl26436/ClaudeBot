package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.field.Reef;
import frc.robot.field.StartPose;
import frc.robot.subsystems.drive.SwerveIOSim;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.testutil.SimHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class DriveSimTest {
    /** Open carpet on the blue side, clear of the reef, the stations and the barge. */
    private static final Pose2d OPEN_FIELD = new Pose2d(2.5, 6.5, Rotation2d.kZero);

    private SwerveIOSim sim;
    private SwerveSubsystem drive;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        sim = new SwerveIOSim();
        drive = new SwerveSubsystem(sim);
        SimHarness.enable();
        SimHarness.run(0.06);
        SimHarness.register(() -> sim.simulate(SimHarness.lastDtSeconds()));
        SimHarness.run(0.14);
    }

    @AfterEach
    void tearDown() {
        SimHarness.reset();
        sim.close();
    }

    /**
     * Stop, teleport, and tell odometry. The wheels must stop first (teleporting zeroes the body's
     * speed but not theirs), and the Pigeon must publish its new yaw before odometry is reset, or
     * the estimate's heading is offset by the teleport.
     */
    private void placeAt(Pose2d pose) {
        drive.stopDriving();
        SimHarness.run(0.5);
        sim.setTruePose(pose);
        SimHarness.run(0.1);
        drive.resetPose(pose);
        SimHarness.run(0.1);
    }

    @Test
    void driveToPointArrives() {
        // The centre start looks straight at face GH, so branch H is a straight-line drive. (Face
        // AB is on the far side of the reef from here; a straight line to it would hit the reef.)
        var target = Reef.scoringPose(Alliance.Blue, Reef.Branch.H);
        placeAt(StartPose.CENTRE.pose(Alliance.Blue));
        drive.driveToPoint(target, 3.0, false);
        SimHarness.run(4);
        assertTrue(drive.atTarget(), "estimate is " + drive.distanceToTargetM() + " m from the target");
        assertEquals(0, sim.truePose().getTranslation().getDistance(target.getTranslation()), 0.05);
        assertEquals(0, sim.truePose().getRotation().minus(target.getRotation()).getDegrees(), 3);
    }

    @Test
    void driveToPointTurnsToTheTargetHeading() {
        placeAt(OPEN_FIELD);
        var target = new Pose2d(3.5, 6.0, Rotation2d.fromDegrees(90));
        drive.driveToPoint(target, 3.0, false);
        SimHarness.run(3);
        assertTrue(drive.atTarget());
        assertEquals(0, sim.truePose().getRotation().minus(target.getRotation()).getDegrees(), 3);
    }

    @Test
    void speedScaleSlowsTheRobot() {
        placeAt(OPEN_FIELD);
        drive.setSpeedScale(1.0);
        drive.setTeleopInput(1, 0, 0);
        SimHarness.run(1.5);
        double fullSpeedM = sim.truePose().getTranslation().getDistance(OPEN_FIELD.getTranslation());

        placeAt(OPEN_FIELD);
        drive.setSpeedScale(0.35);
        drive.setTeleopInput(1, 0, 0);
        SimHarness.run(1.5);
        double scaledM = sim.truePose().getTranslation().getDistance(OPEN_FIELD.getTranslation());

        assertTrue(fullSpeedM > 1.5, "full stick should cover real distance, covered " + fullSpeedM);
        assertTrue(scaledM < fullSpeedM / 2, "scaled run covered " + scaledM + " m against " + fullSpeedM);
    }

    private double speedMps() {
        var v = RobotState.get().speeds();
        return Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
    }

    @Test
    void accelLimitSlowsBothStartAndStop() {
        placeAt(OPEN_FIELD);
        drive.setAccelLimitSource(() -> 2.0);
        drive.setTeleopInput(1, 0, 0);
        SimHarness.run(0.5);
        assertTrue(speedMps() <= 2.0 * 0.5 + 0.3, "half a second at 2 m/s^2 reached " + speedMps() + " m/s");
        assertTrue(speedMps() >= 2.0 * 0.5 - 0.4, "the robot should still be speeding up, at " + speedMps());
        SimHarness.run(0.5);
        double before = speedMps();
        drive.setTeleopInput(0, 0, 0);
        SimHarness.run(0.25);
        assertTrue(speedMps() >= before - 2.0 * 0.25 - 0.3, "stopped faster than 2 m/s^2: " + before + " to " + speedMps());
        drive.setAccelLimitSource(() -> Double.POSITIVE_INFINITY);
    }

    @Test
    void speedScaleAlsoCapsDriveToPoint() {
        placeAt(OPEN_FIELD);
        drive.setSpeedScale(0.35);
        drive.driveToPoint(new Pose2d(7.5, 6.5, Rotation2d.kZero), 3.0, false);
        SimHarness.run(1.0);
        double speed = Math.hypot(RobotState.get().speeds().vxMetersPerSecond, RobotState.get().speeds().vyMetersPerSecond);
        assertTrue(speed <= 3.0 * 0.35 + 0.1, "drive-to-point ran at " + speed + " m/s under a 0.35 scale");
    }

    @Test
    void teleopStickMovesFieldRelative() {
        placeAt(new Pose2d(OPEN_FIELD.getTranslation(), Rotation2d.fromDegrees(90)));
        drive.setTeleopInput(0.5, 0, 0);
        SimHarness.run(1.0);
        Translation2d moved = sim.truePose().getTranslation().minus(OPEN_FIELD.getTranslation());
        assertTrue(moved.getX() > 0.5, "should move along field +x, moved " + moved);
        assertEquals(0, moved.getY(), 0.15, "should not drift sideways, moved " + moved);
        assertEquals(90, sim.truePose().getRotation().getDegrees(), 5, "heading should hold with no turn input");
    }

    /** Clear of the reef and the walls for 1.5 s of driving at stick (0.5, 0.2). */
    private static final Pose2d DIAGONAL_START = new Pose2d(2.0, 6.0, Rotation2d.kZero);

    private double estimateErrorM() {
        return drive.pose().getTranslation().getDistance(sim.truePose().getTranslation());
    }

    /**
     * Odometry follows the truth through straight driving and turning on the spot. Driving while
     * turning drifts a little: every module keeps re-steering, lags and scrubs sideways, which
     * maple-sim models (0.067 m measured over 1.5 s here). The reef position fix corrects that.
     */
    @Test
    void odometryTracksTheTruePose() {
        placeAt(DIAGONAL_START);
        drive.setTeleopInput(0.5, 0.2, 0);
        SimHarness.run(1.5);
        drive.setTeleopInput(0, 0, 0);
        SimHarness.run(0.5);
        assertTrue(sim.truePose().getTranslation().getDistance(DIAGONAL_START.getTranslation()) > 2.5, "it should have moved");
        assertEquals(0, estimateErrorM(), 0.03, "straight: estimate " + drive.pose() + " true " + sim.truePose());

        placeAt(OPEN_FIELD);
        drive.setTeleopInput(0, 0, 0.3);
        SimHarness.run(1.5);
        drive.setTeleopInput(0, 0, 0);
        SimHarness.run(0.5);
        assertTrue(Math.abs(sim.truePose().getRotation().getDegrees()) > 30, "it should have turned");
        assertEquals(0, drive.pose().getRotation().minus(sim.truePose().getRotation()).getDegrees(), 1, "turning heading");
        assertEquals(0, estimateErrorM(), 0.03, "on the spot: estimate " + drive.pose() + " true " + sim.truePose());

        placeAt(DIAGONAL_START);
        drive.setTeleopInput(0.4, 0.15, 0.15);
        SimHarness.run(1.5);
        drive.setTeleopInput(0, 0, 0);
        SimHarness.run(0.5);
        assertEquals(0, estimateErrorM(), 0.1, "turning while driving: estimate " + drive.pose() + " true " + sim.truePose());

        assertEquals(drive.pose(), RobotState.get().pose());
    }

    @Test
    void resetTranslationKeepsTheHeading() {
        placeAt(new Pose2d(OPEN_FIELD.getTranslation(), Rotation2d.fromDegrees(45)));
        drive.resetTranslation(new Translation2d(3, 5));
        SimHarness.run(0.1);
        assertEquals(3, drive.pose().getX(), 0.02);
        assertEquals(5, drive.pose().getY(), 0.02);
        assertEquals(45, drive.pose().getRotation().getDegrees(), 1);
    }

    @Test
    void atTargetNeedsBothPositionAndHeading() {
        placeAt(OPEN_FIELD);
        drive.driveToPoint(new Pose2d(OPEN_FIELD.getTranslation(), Rotation2d.fromDegrees(120)), 3.0, false);
        SimHarness.run(0.04);
        assertFalse(drive.atTarget(), "on the spot but turned the wrong way is not at the target");
    }
}
