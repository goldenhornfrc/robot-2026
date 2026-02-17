package frc.robot.subsystems.hood;

public class HoodConstants {
  // Motor configuration
  public static final int HOOD_MOTOR_ID = 15;
  public static final double HOOD_SUPPLY_CURRENT_LIMIT = 30.0;
  public static final double HOOD_STATOR_CURRENT_LIMIT = 80.0;
  public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = (52.0 / 8.0) * 16.0;

  // Motion magic parameters
  public static final double kHoodStartingPos = 0.0;
  public static final double kHoodExtendLimit = 0.0;
  public static final double kHoodCruiseVel = 0.0;
  public static final double kHoodAccel = 0.0;

  // PID constants (placeholder values)
  public static final double kP = 0.0;
  public static final double kD = 0.0;
  public static final double kG = 0.0;
}
