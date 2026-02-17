package frc.robot.subsystems.intake;

public class IntakeConstants {
  // Device IDs
  public static final int MOTOR_ID = 26;

  // Current limits (Amps)
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 80.0;

  // Sensor ratio (gear reduction)
  public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

  // Pivot constants
  public static double intakePivotStartingPos = 10.0;
  public static double intakePivotExtendLimitPos = 13.5;

  public static double intakePivotAccel = 10.0;
  public static double intakePivotCruiseVel = 3.0;

  public static double kIntakePivotAllowableErrorDegrees = 1.2;
  public static final double kP = 55.0;
  public static final double kD = 1.0;
  public static final double kG = 0.5;
}
