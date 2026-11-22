package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.Reef;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

/**
 * Simulated Limelight: projects every reef tag through {@link CameraModel} from the robot's true
 * pose and reports the largest one in view, as the Limelight reports its primary target.
 */
public class VisionIOSim implements VisionIO {
    /** Pixel noise on each sighting. Zero keeps tests repeatable. */
    private static final double NOISE_PX = 0.0;

    private final Supplier<Pose2d> truePose;
    private final CameraModel cam;
    private final Random rng = new Random(4206);

    public VisionIOSim(Supplier<Pose2d> truePose, CameraModel cam) {
        this.truePose = truePose;
        this.cam = cam;
    }

    @Override
    public void updateInputs(VisionIOInputs in) {
        Pose2d robot = truePose.get();
        Optional<TagObservation> best = Optional.empty();
        for (int id : Reef.allTagIds()) {
            var seen = cam.project(robot, id, Reef.anyTagPose3d(id), NOISE_PX, rng);
            if (seen.isPresent() && (best.isEmpty() || seen.get().heightPx() > best.get().heightPx())) {
                best = seen;
            }
        }
        in.connected = true;
        in.hasTarget = best.isPresent();
        in.tagId = best.map(TagObservation::tagId).orElse(-1);
        in.txDeg = best.map(TagObservation::txDeg).orElse(0.0);
        in.heightPx = best.map(TagObservation::heightPx).orElse(0.0);
    }
}
