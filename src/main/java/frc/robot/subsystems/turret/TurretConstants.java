package frc.robot.subsystems.turret;

public class TurretConstants {
  // Motor configuration
  public static final int TURRET_MOTOR_ID = 6;
  public static final double TURRET_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double TURRET_STATOR_CURRENT_LIMIT = 50.0;
  public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = 40.0;

  // Motion magic parameters
  public static final double kTurretCCWLimit = 77.0;
  public static final double kTurretCWLimit = -304.0;
  public static final double kTurretCruiseVel = 20.0;
  public static final double kTurretAccel = 40.0;

  public static final double kTurretResetAngle = 77.7;
  // PID constants (placeholder values)
  public static final double kP = 125.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kV = 0.0339; // 0.0209
  public static final double kA = 0.0;
}
