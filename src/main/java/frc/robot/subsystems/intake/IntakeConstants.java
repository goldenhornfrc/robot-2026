package frc.robot.subsystems.intake;

public class IntakeConstants {
  // Intake Motor constants
  public static final int INTAKE_MOTOR_ID = 26;
  public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double INTAKE_STATOR_CURRENT_LIMIT = 80.0;
  public static final double INTAKE_SENSOR_TO_MECHANISM_RATIO = 1.0;

  // Intake Pivot Motor constants
  public static final int INTAKE_PIVOT_MOTOR_ID = 25;
  public static final double INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double INTAKE_PIVOT_STATOR_CURRENT_LIMIT = 70.0;
  public static final double INTAKE_PIVOT_SENSOR_TO_MECHANISM_RATIO = (40.0 / 12.0) * 6.222;

  // Pivot motion magic constants
  public static double intakePivotStartingPos = 100.0;
  public static double intakePivotExtendLimitPos = -1.0;
  public static double intakePivotAccel = 10.0;
  public static double intakePivotCruiseVel = 3.0;

  // Pivot PID constants
  public static double kIntakePivotAllowableErrorDegrees = 1.2;
  public static final double kP = 55.0;
  public static final double kD = 1.0;
  public static final double kG = 0.5;
}
