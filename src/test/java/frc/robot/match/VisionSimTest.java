package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.field.Reef;
import frc.robot.subsystems.drive.SwerveIOSim;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.CameraModel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.testutil.SimHarness;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class VisionSimTest {
    private SwerveIOSim sim;
    private SwerveSubsystem drive;
    private Vision vision;

    @BeforeEach
    void setUp() {
        SimHarness.reset();
        setAlliance(AllianceStationID.Blue1);
        sim = new SwerveIOSim();
        drive = new SwerveSubsystem(sim);
        vision = new Vision(new VisionIOSim(sim::truePose, CameraModel.limelight4()), drive);
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

    private static void setAlliance(AllianceStationID station) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
    }

    private static Pose2d oneMetreFromFace(int face) {
        return Reef.tagPose(Alliance.Blue, face).transformBy(new Transform2d(1.0, 0, Rotation2d.k180deg));
    }

    private static Pose2d shifted(Pose2d p, double dx, double dy) {
        return new Pose2d(p.getTranslation().plus(new Translation2d(dx, dy)), p.getRotation());
    }

    /** Teleport and let the Pigeon publish the new yaw before telling odometry anything. */
    private void placeAt(Pose2d pose) {
        sim.setTruePose(pose);
        SimHarness.run(0.1);
        drive.resetPose(pose);
        SimHarness.run(0.1);
    }

    private double errorM() {
        return drive.pose().getTranslation().getDistance(sim.truePose().getTranslation());
    }

    @Test
    void fixCorrectsAnOdometryError() {
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 0.25, -0.15));
        vision.setTargetFace(Optional.of(0));
        SimHarness.run(0.3);
        assertEquals(0, errorM(), 0.05);
        assertTrue(vision.lastFix().isPresent());
    }

    @Test
    void fixWorksOnEveryFace() {
        for (int face = 0; face < 6; face++) {
            placeAt(oneMetreFromFace(face));
            drive.resetPose(shifted(sim.truePose(), -0.2, 0.2));
            vision.setTargetFace(Optional.of(face));
            SimHarness.run(0.3);
            assertEquals(0, errorM(), 0.05, "face " + face);
        }
    }

    @Test
    void ignoresTheWrongFace() {
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 0.25, 0));
        vision.setTargetFace(Optional.of(3));
        SimHarness.run(0.3);
        assertTrue(vision.lastFix().isEmpty());
        assertEquals(0.25, errorM(), 0.03, "the error is left alone");
    }

    @Test
    void noFixWhenNoFaceIsSet() {
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 0.25, -0.15));
        vision.setTargetFace(Optional.empty());
        SimHarness.run(0.3);
        assertTrue(vision.lastFix().isEmpty());
        assertEquals(Math.hypot(0.25, 0.15), errorM(), 0.03);
    }

    @Test
    void rejectsAFixFarFromTheEstimate() {
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 1.2, 0.4));
        vision.setTargetFace(Optional.of(0));
        SimHarness.run(0.3);
        assertTrue(vision.lastFix().isEmpty(), "a fix over 1 m from the estimate is not trusted");
    }

    @Test
    void redAllianceUsesRedTags() {
        setAlliance(AllianceStationID.Red1);
        var nearBlueFace = oneMetreFromFace(0);
        placeAt(nearBlueFace);
        drive.resetPose(shifted(sim.truePose(), 0.2, 0));
        vision.setTargetFace(Optional.of(0)); // red face 0 is across the field: the blue tag in view is not it
        SimHarness.run(0.3);
        assertTrue(vision.lastFix().isEmpty());
    }

    @Test
    void unknownAllianceMakesNoFix() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 0.2, 0));
        vision.setTargetFace(Optional.of(0));
        SimHarness.run(0.3);
        assertTrue(vision.lastFix().isEmpty());
    }

    /** A camera reporting the right tag with a degenerate size, as a glitching pipeline can. */
    private static final class DegenerateIO implements VisionIO {
        final double heightPx;

        DegenerateIO(double heightPx) {
            this.heightPx = heightPx;
        }

        @Override
        public void updateInputs(VisionIOInputs in) {
            in.connected = true;
            in.hasTarget = true;
            in.tagId = 18;
            in.txDeg = 0;
            in.heightPx = heightPx;
        }
    }

    @Test
    void rejectsDegenerateSightings() {
        placeAt(oneMetreFromFace(0));
        var before = drive.pose();
        for (double px : new double[] {0, 1, Double.NaN, Double.POSITIVE_INFINITY}) {
            var bad = new Vision(new DegenerateIO(px), drive);
            bad.setTargetFace(Optional.of(0));
            SimHarness.run(0.1);
            assertTrue(bad.lastFix().isEmpty(), "heightPx " + px);
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(bad);
        }
        assertEquals(0, drive.pose().getTranslation().getDistance(before.getTranslation()), 0.02);
    }
}
