package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
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
    void aFixFarFromTheEstimateIsNotTrustedAtOnce() {
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 1.2, 0.4));
        vision.setTargetFace(Optional.of(0));
        SimHarness.run(0.2);
        assertTrue(vision.lastFix().isEmpty(), "a fix over 1 m from the estimate is not trusted on sight");
        assertTrue(vision.doubtsThePosition(), "but the position is in doubt");
    }

    /**
     * A bad sighting jumps about; a wrong position estimate does not. When the camera keeps saying the
     * same thing, it is the estimate that is wrong.
     */
    @Test
    void aFarFixThatKeepsAgreeingWithItselfIsBelieved() {
        placeAt(oneMetreFromFace(0));
        drive.resetPose(shifted(sim.truePose(), 1.2, 0.4));
        vision.setTargetFace(Optional.of(0));
        SimHarness.run(0.8);
        assertTrue(vision.lastFix().isPresent());
        assertEquals(0, errorM(), 0.06);
        assertFalse(vision.doubtsThePosition());
    }

    /** A camera whose reported tag size the test sets each loop. */
    private static final class ScriptedIO implements VisionIO {
        java.util.function.IntFunction<Double> heightPx;
        java.util.function.IntPredicate seen = i -> true;
        int loop = 0;

        @Override
        public void updateInputs(VisionIOInputs in) {
            in.connected = true;
            in.hasTarget = seen.test(loop);
            in.tagId = Reef.tagId(Alliance.Blue, 0);
            in.txDeg = 0;
            in.heightPx = heightPx.apply(loop);
            loop++;
        }
    }

    private void withCamera(ScriptedIO io, java.util.function.Consumer<Vision> body) {
        var v = new Vision(io, drive); // registered with the scheduler, so the harness runs its periodic
        try {
            v.setTargetFace(Optional.of(0));
            body.accept(v);
        } finally {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(v);
        }
    }

    /** The robot really is 3 m from the tag; this camera says nearer, by a different amount each loop. */
    @Test
    void sightingsThatJumpAboutAreNeverBelieved() {
        placeAt(Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(3.0, 0, Rotation2d.k180deg)));
        var io = new ScriptedIO();
        io.heightPx = i -> i % 2 == 0 ? 120.0 : 200.0;
        withCamera(io, v -> {
            SimHarness.run(2.0);
            assertTrue(v.lastFix().isEmpty());
            assertEquals(0, errorM(), 0.05);
        });
    }

    @Test
    void aBurstOfBadSightingsIsNotEnough() {
        placeAt(Reef.tagPose(Alliance.Blue, 0).transformBy(new Transform2d(3.0, 0, Rotation2d.k180deg)));
        var io = new ScriptedIO();
        io.heightPx = i -> 120.0;
        io.seen = i -> i % 20 < 8; // 0.16 s at a time, then nothing for a while
        withCamera(io, v -> {
            SimHarness.run(2.0);
            assertTrue(v.lastFix().isEmpty());
            assertEquals(0, errorM(), 0.05);
        });
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
