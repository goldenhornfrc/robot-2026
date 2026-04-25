package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  // Device IDs
  public static final int LEFT_MOTOR_ID = 11;
  public static final int RIGHT_MOTOR_ID = 15;

  // Current limits (Amps)
  public static final double SUPPLY_CURRENT_LIMIT = 60.0;
  public static final double STATOR_CURRENT_LIMIT = 120.0;

  // Sensor ratio (gear reduction)
  public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

  // PID and feedforward gains
  public static final double KP = 0.3;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.25;
  public static final double KV = 0.10011; // 0.108
  public static final double KA = 0.009486;

  public static final double RPM_TOLERANCE = 75.0;

  private ShooterConstants() {}
}
