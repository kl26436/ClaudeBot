package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCommands;
import frc.robot.subsystems.climber.Climber.State;
import frc.robot.testutil.AutoReplay;
import frc.robot.testutil.RobotFixture;
import frc.robot.testutil.SimHarness;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

/** Records one whole climb for the playbook site's animations. Runs only when {@code replayDir} is set. */
@Tag("sim")
@EnabledIfSystemProperty(named = "replayDir", matches = ".+")
class ClimbReplayTest {
    private static final int TICKS_PER_SAMPLE = 5;

    private final List<String> samples = new ArrayList<>();
    private final List<String> events = new ArrayList<>();
    private int ticks = 0;
    private State last = null;

    private void run(double seconds, State until) {
        for (int i = 0; i < seconds / SimHarness.DT && RobotFixture.get().climber().state() != until; i++) {
            var r = RobotFixture.get();
            if (ticks % TICKS_PER_SAMPLE == 0) {
                samples.add(String.format(
                        Locale.ROOT,
                        "[%.1f,%.1f,%.1f,%.1f,%.1f]",
                        ticks * SimHarness.DT,
                        r.sim().trueCarriageIn(),
                        r.sim().trueWristDeg(),
                        r.sim().trueForkDeg(),
                        r.sim().trayDeg()));
            }
            State now = r.climber().state();
            if (now != last) {
                events.add(String.format(Locale.ROOT, "{\"t\":%.1f,\"what\":\"%s\"}", ticks * SimHarness.DT, now));
                last = now;
            }
            SimHarness.run(SimHarness.DT);
            ticks++;
        }
    }

    @Test
    void record() throws Exception {
        var r = RobotFixture.reset(new Pose2d(7.0, 6.2, Rotation2d.kZero), false);
        var climb = ClimbCommands.climb(r);
        run(0.5, null);
        CommandScheduler.getInstance().schedule(climb);
        run(12, State.DEPLOYED);
        climb.cancel();
        run(2.0, null); // backing onto the cage
        r.sim().setCageCaptured(true);
        CommandScheduler.getInstance().schedule(climb);
        run(6, State.CLIMBED);
        climb.cancel();
        run(1.0, null);
        assertEquals(State.CLIMBED, r.climber().state());
        Files.createDirectories(AutoReplay.folder());
        Files.writeString(
                AutoReplay.folder().resolve("climb.json"),
                "{\n\"label\":\"Deep climb\",\n" + AutoReplay.stamp() + ",\n"
                        + "\"columns\":[\"t_s\",\"carriage_in\",\"wrist_deg\",\"fork_deg\",\"tray_deg\"],\n"
                        + "\"events\":[" + String.join(",", events) + "],\n"
                        + "\"samples\":[\n" + String.join(",\n", samples) + "\n]}\n");
    }
}
