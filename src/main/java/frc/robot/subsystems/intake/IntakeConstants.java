package frc.robot.subsystems.intake;

public class IntakeConstants {
  // Intake Motor constants
  public static final int INTAKE_MOTOR_ID = 43;
  public static final int INTAKE_MOTOR2_ID = 44; // TODO: update with actual ID
  public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double INTAKE_STATOR_CURRENT_LIMIT = 80.0;
  public static final double INTAKE_SENSOR_TO_MECHANISM_RATIO = 1.0;

  // Intake Deploy Motor constants
  public static final int INTAKE_DEPLOY_MOTOR_ID = 5;
  public static final double INTAKE_DEPLOY_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double INTAKE_DEPLOY_STATOR_CURRENT_LIMIT = 30.0;
  public static final double INTAKE_DEPLOY_SENSOR_TO_MECHANISM_RATIO = (43.0 / 20.0) * 3.0;

  // Deploy motion magic constants
  public static double intakeDeployStartingPos = 0.0;
  public static double intakeDeployExtendLimitPos = 3.63;
  public static double intakeDeployAccel = 40.0;
  public static double intakeDeployCruiseVel = 30.0;

  // Deploy PID constants
  public static double kIntakeDeployAllowableErrorRotations = 0.1; // TODO: will have to be tuned
  public static final double kP = 50.0;
  public static final double kD = 0.0;
}
