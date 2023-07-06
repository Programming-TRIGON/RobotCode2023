package frc.trigon.robot.constants;

public class ConfigurationConstants {
  public static final RobotType ROBOT_TYPE = RobotType.TRIHARD;
  public static final boolean IS_REPLAY = false;

  public enum RobotType {
    TRIHARD,
    TESTING,
    // TODO: none of this bruh
    SIMULATION
  }
}