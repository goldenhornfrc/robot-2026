package frc.robot.subsystems.spindexer;

public final class SpindexerConstants {
  // Device IDs
  public static final int MOTOR_ID = 60;

  // Current limits (Amps)
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 80.0;

  // Sensor ratio (gear reduction)
  public static final double SENSOR_TO_MECHANISM_RATIO = 5.0;

  // Default PID & FF parameters
  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;

  public static final double KS = 0.0;
  public static final double KV = 0.0;
  public static final double KA = 0.0;

  private SpindexerConstants() {}
}
