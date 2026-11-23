package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Remembers, across a robot-code restart, that the climb has begun and in which match. Nothing on the
 * robot can tell a hanging fork (88 degrees) from a stowed one (90), so the code has to remember.
 */
public interface CommitMemory {
    /** The match the climb was begun in, or empty if it was not. */
    String load();

    void save(String match);

    /** The match being played now, or {@link #PRACTICE} with no field attached. */
    String currentMatch();

    String PRACTICE = "practice";

    /** Kept on the roboRIO's disk. */
    static CommitMemory onDisk() {
        return new CommitMemory() {
            private static final String KEY = "Climber/CommittedIn";

            @Override
            public String load() {
                return Preferences.getString(KEY, "");
            }

            @Override
            public void save(String match) {
                Preferences.setString(KEY, match);
            }

            @Override
            public String currentMatch() {
                if (!DriverStation.isFMSAttached()) {
                    return PRACTICE;
                }
                return DriverStation.getEventName() + " " + DriverStation.getMatchType() + " "
                        + DriverStation.getMatchNumber() + "." + DriverStation.getReplayNumber();
            }
        };
    }

    /** Kept only as long as this object, for simulation and tests. */
    static CommitMemory inMemory(String saved) {
        return new CommitMemory() {
            private String match = saved;

            @Override
            public String load() {
                return match;
            }

            @Override
            public void save(String m) {
                match = m;
            }

            @Override
            public String currentMatch() {
                return PRACTICE;
            }
        };
    }
}
