package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * A Limelight 4 running an AprilTag pipeline, read from NetworkTables: {@code tv} (target valid),
 * {@code tid} (tag ID), {@code tx} (degrees, right positive), {@code tcornxy} (corner pixels) and
 * {@code hb} (heartbeat, which counts up while the camera is running).
 */
public class VisionIOLimelight implements VisionIO {
    /** The camera counts as disconnected when its heartbeat has not changed for this long. */
    private static final double HEARTBEAT_TIMEOUT_S = 0.5;

    private final DoubleSubscriber tv;
    private final DoubleSubscriber tid;
    private final DoubleSubscriber tx;
    private final DoubleArraySubscriber corners;
    private final DoubleSubscriber heartbeat;
    private double lastHeartbeat = Double.NaN;
    private double lastHeartbeatChangeS = Double.NEGATIVE_INFINITY;

    public VisionIOLimelight(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        tv = table.getDoubleTopic("tv").subscribe(0);
        tid = table.getDoubleTopic("tid").subscribe(-1);
        tx = table.getDoubleTopic("tx").subscribe(0);
        corners = table.getDoubleArrayTopic("tcornxy").subscribe(new double[0]);
        heartbeat = table.getDoubleTopic("hb").subscribe(Double.NaN);
    }

    @Override
    public void updateInputs(VisionIOInputs in) {
        double now = Timer.getFPGATimestamp();
        double hb = heartbeat.get();
        if (!Double.isNaN(hb) && hb != lastHeartbeat) {
            lastHeartbeat = hb;
            lastHeartbeatChangeS = now;
        }
        in.connected = now - lastHeartbeatChangeS <= HEARTBEAT_TIMEOUT_S;
        in.hasTarget = in.connected && tv.get() >= 1;
        in.tagId = (int) tid.get();
        in.txDeg = tx.get();
        in.heightPx = tagHeightPx(corners.get());
    }

    /**
     * Height of the primary tag in pixels: the mean length of its two vertical edges, from its four
     * corners (x0, y0, ..., x3, y3). NaN without four corners.
     */
    public static double tagHeightPx(double[] xy) {
        if (xy.length < 8) {
            return Double.NaN;
        }
        // Sort the four corners by x: the two leftmost form the left edge, the others the right edge.
        Integer[] order = {0, 1, 2, 3};
        java.util.Arrays.sort(order, (a, b) -> Double.compare(xy[2 * a], xy[2 * b]));
        double left = Math.abs(xy[2 * order[0] + 1] - xy[2 * order[1] + 1]);
        double right = Math.abs(xy[2 * order[2] + 1] - xy[2 * order[3] + 1]);
        return (left + right) / 2;
    }
}
