package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  // Device IDs
  public static final int LEFT_MOTOR_ID = 12;
  public static final int RIGHT_MOTOR_ID = 11;

  // Current limits (Amps)
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 80.0;

  // Sensor ratio (gear reduction)
  public static final double SENSOR_TO_MECHANISM_RATIO = 18.0 / 15.0;

  // PID and feedforward gains
  public static final double KP = 0.45;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.0;
  public static final double KV = 0.115;
  public static final double KA = 0.0;

  // RPM tolerance for setpoint checking
  public static final double RPM_TOLERANCE = 50.0;

  private ShooterConstants() {}
}
