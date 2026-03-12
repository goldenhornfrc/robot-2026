package frc.robot.subsystems.hood;

public class HoodConstants {
  // Motor configuration
  public static final int HOOD_MOTOR_ID = 12;
  public static final double HOOD_SUPPLY_CURRENT_LIMIT = 30.0;
  public static final double HOOD_STATOR_CURRENT_LIMIT = 40.0;
  public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = (64.0 / 8.0) * (295.0 / 30.0);

  // Motion magic parameters
  public static final double kHoodStartingPos = 0.0;
  public static final double kHoodExtendLimit = 25.5;
  public static final double kHoodCruiseVel = 20;
  public static final double kHoodAccel = 50;

  // PID constants (placeholder values)
  public static final double kP = 450.0;
  public static final double kD = 0.0;
  public static final double kS = 1.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
}
