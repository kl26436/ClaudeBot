package frc.robot.testutil;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Auto;
import frc.robot.autos.AutoChooser;
import frc.robot.sim.ScoreJudge.CoralScore;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Records where a simulated auto really was, ten times a second, and writes it as JSON for the playbook
 * site's replays. Off unless the {@code replayDir} system property names a folder.
 */
public final class AutoReplay {
    public static final double STEP_S = 0.1;

    private final List<String> samples = new ArrayList<>();
    private int algaeInNet = 0;
    private double netTimeS = Double.NaN;

    /** The folder to write to, or null when replays are off. */
    public static Path folder() {
        String dir = System.getProperty("replayDir", "");
        return dir.isEmpty() ? null : Path.of(dir);
    }

    /** One row: time, x, y (metres, blue origin), heading, carriage (in), wrist (degrees), coral, algae. */
    public void sample(AutoHarness h, double tS) {
        Pose2d p = h.sim().truePose();
        var s = h.robot().superstructure();
        samples.add(String.format(
                Locale.ROOT,
                "[%.1f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%d]",
                tS,
                p.getX(),
                p.getY(),
                p.getRotation().getDegrees(),
                h.sim().trueCarriageIn(),
                h.sim().trueWristDeg(),
                s.hasCoral() ? 1 : 0,
                s.hasAlgae() ? 1 : 0));
        if (h.sim().algaeInNet() > algaeInNet) {
            algaeInNet = h.sim().algaeInNet();
            netTimeS = tS;
        }
    }

    public void write(Path folder, Auto auto, List<CoralScore> scored) throws IOException {
        List<String> events = new ArrayList<>();
        for (CoralScore c : scored) {
            events.add(String.format(
                    Locale.ROOT, "{\"t\":%.1f,\"what\":\"coral\",\"branch\":\"%s\",\"level\":\"%s\"}", c.timeS(), c.branch(), c.level()));
        }
        if (algaeInNet > 0) {
            events.add(String.format(Locale.ROOT, "{\"t\":%.1f,\"what\":\"net\"}", netTimeS));
        }
        String label = AutoChooser.PROGRAMS.stream().filter(p -> p.auto() == auto).findFirst().orElseThrow().label();
        String json = "{\n\"auto\":\"" + auto + "\",\"label\":\"" + label + "\",\"alliance\":\"Blue\",\n"
                + "\"columns\":[\"t_s\",\"x_m\",\"y_m\",\"heading_deg\",\"carriage_in\",\"wrist_deg\",\"coral\",\"algae\"],\n"
                + "\"events\":[" + String.join(",", events) + "],\n"
                + "\"samples\":[\n" + String.join(",\n", samples) + "\n]}\n";
        Files.createDirectories(folder);
        Files.writeString(folder.resolve(auto.name().toLowerCase(Locale.ROOT) + ".json"), json);
    }
}
