package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator {

  private static LaunchCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.4 / 0.02));
  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (1.5 / 0.02));

  private double lastHoodAngle;
  private Rotation2d lastTurretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  private static final double DRAG_CONSTANT = 0.75;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double phaseDelay;

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.35;
    maxDistance = 5.64;
    phaseDelay = 0.03;

    hoodAngleMap.put(1.35, 0.0);
    hoodAngleMap.put(1.75, 2.0);
    hoodAngleMap.put(2.05, 5.0);
    hoodAngleMap.put(2.38, 8.0);
    hoodAngleMap.put(2.75, 10.0);
    hoodAngleMap.put(3.1, 13.0);
    hoodAngleMap.put(3.5, 13.0);
    hoodAngleMap.put(4.0, 15.0);
    hoodAngleMap.put(4.4, 16.0);
    hoodAngleMap.put(4.83, 19.0);
    hoodAngleMap.put(5.0, 21.0);
    hoodAngleMap.put(5.64, 24.0);

    flywheelSpeedMap.put(1.35, 2700.0);
    flywheelSpeedMap.put(1.75, 2700.0);
    flywheelSpeedMap.put(2.055, 2700.0);
    flywheelSpeedMap.put(2.38, 2725.0);
    flywheelSpeedMap.put(2.75, 2750.0);
    flywheelSpeedMap.put(3.1, 2900.0);
    flywheelSpeedMap.put(3.5, 3000.0);
    flywheelSpeedMap.put(4.0, 3075.0);
    flywheelSpeedMap.put(4.4, 3200.0);
    flywheelSpeedMap.put(4.83, 3300.0);
    flywheelSpeedMap.put(5.0, 3400.0);
    flywheelSpeedMap.put(5.64, 3600.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public void setHoodAngleOffsetDeg(double offset) {
    hoodAngleOffsetDeg = offset;
  }

  public double getHoodAngleOffsetDeg() {
    return hoodAngleOffsetDeg;
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters() {

    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d();
    // AllianceFlipUtil.apply();

    Pose2d turretPosition =
        estimatedPose.transformBy(GeomUtil.toTransform2d(VisionConstants.ROBOT_TO_TURRET));
    Logger.recordOutput("LaunchCalculator/turretPosition", turretPosition);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
    Logger.recordOutput("LaunchCalculator/turretToTargetDistance", turretToTargetDistance);

    /*
        // 1. Get the translation offset of the turret relative to the robot center
        Translation2d robotToTurretTranslation =
            VisionConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d();

        // 2. Rotate this offset into field space using the robot's current heading
        Translation2d fieldRelativeOffset =
            robotToTurretTranslation.rotateBy(estimatedPose.getRotation());
    */
    // 3. Calculate tangential velocity (omega x r)
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
    double omega = robotVelocity.omegaRadiansPerSecond;
    double robotAngle = estimatedPose.getRotation().getRadians();
    /*
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            - omega
                * (robotToTurretTranslation.getX() * Math.sin(robotAngle)
                    + robotToTurretTranslation.getY() * Math.cos(robotAngle));

    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + omega
                * (robotToTurretTranslation.getX() * Math.cos(robotAngle)
                    - robotToTurretTranslation.getY() * Math.sin(robotAngle));
    */
    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);
    Pose2d lookaheadPose = turretPosition;
    double lookaheadturretToTargetDistance = turretToTargetDistance;

    for (int i = 0; i < 20; i++) {
      // 1. Get the actual time of flight needed to reach the target
      timeOfFlight = timeOfFlightMap.get(lookaheadturretToTargetDistance);

      // 2. Calculate the drag-adjusted effective time of flight
      double effectiveTimeOfFlight =
          DRAG_CONSTANT * (1.0 - Math.exp(-timeOfFlight / DRAG_CONSTANT));

      // 3. Apply the EFFECTIVE time of flight to the robot's displacement
      Pose2d lookaheadRobotPose =
          estimatedPose.exp(
              new Twist2d(
                  robotRelativeVelocity.vxMetersPerSecond * effectiveTimeOfFlight,
                  robotRelativeVelocity.vyMetersPerSecond * effectiveTimeOfFlight,
                  robotRelativeVelocity.omegaRadiansPerSecond * effectiveTimeOfFlight));

      lookaheadPose =
          lookaheadRobotPose.transformBy(GeomUtil.toTransform2d(VisionConstants.ROBOT_TO_TURRET));

      lookaheadturretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    // Vector from the predicted turret position to the target
    Rotation2d turretTargetAngle =
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(estimatedPose.getRotation());
    Logger.recordOutput("LaunchCalculator/Target", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput("LaunchCalculator/TurretTargetDegrees", turretTargetAngle.getDegrees());
    hoodAngle = hoodAngleMap.get(lookaheadturretToTargetDistance);

    if (lastTurretAngle == null) lastTurretAngle = turretTargetAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    turretVelocity =
        turretAngleFilter.calculate(turretTargetAngle.minus(lastTurretAngle).getRadians() / 0.02);

    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    lastTurretAngle = turretTargetAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new LaunchingParameters(
            lookaheadturretToTargetDistance >= minDistance
                && lookaheadturretToTargetDistance <= maxDistance,
            turretTargetAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelSpeedMap.get(lookaheadturretToTargetDistance));

    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadturretToTargetDistance);
    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
