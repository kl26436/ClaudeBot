package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.field.Reef;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Corrects the drivetrain's position from the reef tag the robot is driving toward. A fix is applied
 * only while a target face is set, when the camera sees that face's tag for our alliance, and when
 * the fix lands within {@link #MAX_JUMP_M} of the current estimate. A fix farther out than that has to
 * prove itself first: a bad sighting jumps about, while a wrong estimate stays wrong by the same amount,
 * so a far fix that keeps agreeing with itself for {@link #FAR_AGREE_S} is believed. The heading always
 * comes from the gyro; only the translation is replaced.
 */
public class Vision extends SubsystemBase {
    /** A fix farther than this from the estimate is more likely a bad sighting than a real error. */
    static final double MAX_JUMP_M = 1.0;

    /** A far fix is believed after saying the same thing, to within {@link #FAR_AGREE_M}, for this long. */
    static final double FAR_AGREE_S = 0.3; // PLACEHOLDER(tuning)

    static final double FAR_AGREE_M = 0.25; // PLACEHOLDER(tuning)

    /** A far fix seen this recently leaves the position in doubt. */
    static final double DOUBT_S = 0.25;

    /**
     * Smallest tag image the fix trusts. A reef tag at the camera's 4 m range is about 33 px tall;
     * anything near zero would put the robot kilometres away.
     */
    static final double MIN_HEIGHT_PX = 8;

    private static final String FAR = "fix too far from the estimate";

    private final VisionIO io;
    private final SwerveSubsystem drive;
    private final CameraModel cam = CameraModel.limelight4();
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private Optional<Integer> targetFace = Optional.empty();
    private Optional<Translation2d> lastFix = Optional.empty();
    /** How far, and which way, the far fixes of the present run put the robot from its estimate. */
    private Translation2d farOffset = null;

    private double farSinceS = Double.NaN;
    private double lastFarS = Double.NaN;

    public Vision(VisionIO io, SwerveSubsystem drive) {
        this.io = io;
        this.drive = drive;
    }

    /** The reef face being driven to, or empty for no fixes. Changing it forgets the last fix. */
    public void setTargetFace(Optional<Integer> face) {
        if (!face.equals(targetFace)) {
            lastFix = Optional.empty();
            forgetFarFixes();
            lastFarS = Double.NaN;
        }
        targetFace = face;
    }

    /**
     * True while the camera sees the target's tag somewhere the position estimate cannot explain, and it
     * is not settled yet which of them is wrong. Not a moment to let go of a game piece.
     */
    public boolean doubtsThePosition() {
        return Timer.getFPGATimestamp() - lastFarS <= DOUBT_S; // NaN: never doubted
    }

    private void forgetFarFixes() {
        farOffset = null;
        farSinceS = Double.NaN;
    }

    /** Count a far fix toward being believed; true once the run of them has agreed for long enough. */
    private boolean farFixHolds(Translation2d offset) {
        double now = Timer.getFPGATimestamp();
        lastFarS = now;
        if (farOffset == null || offset.getDistance(farOffset) > FAR_AGREE_M) {
            farOffset = offset;
            farSinceS = now;
        }
        return now - farSinceS >= FAR_AGREE_S;
    }

    /** The last fix applied for the current target face. */
    public Optional<Translation2d> lastFix() {
        return lastFix;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        boolean far = farOffset != null;
        String rejected = evaluate();
        if (far && !rejected.equals(FAR)) {
            forgetFarFixes(); // a run is unbroken: one loop without a far fix ends it
        }
        Logger.recordOutput("Vision/Rejected", rejected);
        Logger.recordOutput("Vision/Fix", lastFix.map(t -> new double[] {t.getX(), t.getY()}).orElse(new double[0]));
    }

    /** Apply a fix if this loop's sighting earns one; returns why not, or "" when one was applied. */
    private String evaluate() {
        if (targetFace.isEmpty()) {
            return "no target face";
        }
        if (!inputs.connected || !inputs.hasTarget) {
            return "no tag in view";
        }
        Optional<Alliance> alliance = RobotState.get().alliance();
        if (alliance.isEmpty()) {
            return "alliance unknown";
        }
        int face = targetFace.get();
        if (inputs.tagId != Reef.tagId(alliance.get(), face)) {
            return "tag " + inputs.tagId + " is not face " + face;
        }
        if (!Double.isFinite(inputs.heightPx) || !Double.isFinite(inputs.txDeg) || inputs.heightPx < MIN_HEIGHT_PX) {
            return "degenerate sighting";
        }
        var estimate = drive.pose();
        var fix = SingleTagFix.robotPosition(
                new TagObservation(inputs.tagId, inputs.txDeg, inputs.heightPx),
                cam,
                estimate.getRotation(),
                Reef.tagPose3d(alliance.get(), face));
        if (!Double.isFinite(fix.getX()) || !Double.isFinite(fix.getY())) {
            return "fix not finite";
        }
        Translation2d offset = fix.minus(estimate.getTranslation());
        if (offset.getNorm() > MAX_JUMP_M && !farFixHolds(offset)) {
            return FAR;
        }
        lastFarS = Double.NaN;
        drive.resetTranslation(fix);
        lastFix = Optional.of(fix);
        return "";
    }
}
