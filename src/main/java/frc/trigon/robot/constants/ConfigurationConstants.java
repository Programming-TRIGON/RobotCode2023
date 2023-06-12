package frc.trigon.robot.constants;

public final class ConfigurationConstants {
  public static final Mode CURRENT_MODE = Mode.SIM;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}