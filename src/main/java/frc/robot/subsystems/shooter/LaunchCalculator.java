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
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator {

  private static LaunchCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.4 / 0.02));
  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (3)); // 1.5 / 0.02

  private double lastHoodAngle;
  private Rotation2d lastTurretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  private LoggedTunableNumber DRAG_CONSTANT =
      new LoggedTunableNumber("LaunchCalculator/DragConstant", 1.8);

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

    timeOfFlightMap.put(5.7, 1.2);
    timeOfFlightMap.put(5.38, 1.18);
    timeOfFlightMap.put(4.45, 1.21);
    timeOfFlightMap.put(3.13, 1.08);
    timeOfFlightMap.put(2.43, 1.10);
    timeOfFlightMap.put(1.85, 1.09);
    timeOfFlightMap.put(1.36, 1.08);
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

    // =========================================================================
    // PHASE 1: PRE-LAUNCH PREDICTION (Mechanical Phase Delay)
    // The ball is still in the robot. The robot continues to move and spin.
    // =========================================================================
    Pose2d launchRobotPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Pose2d launchTurretPose =
        launchRobotPose.transformBy(GeomUtil.toTransform2d(VisionConstants.ROBOT_TO_TURRET));

    Logger.recordOutput("LaunchCalculator/LaunchTurretPose", launchTurretPose);

    // =========================================================================
    // PHASE 2: INSTANTANEOUS LAUNCH VELOCITY
    // Calculate how fast the physical turret is moving through field space
    // at the exact millisecond the ball leaves the barrel.
    // =========================================================================
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    double omega = fieldVelocity.omegaRadiansPerSecond;

    // Rotate the turret offset into the field frame using the predicted launch heading
    Translation2d robotToTurretField =
        VisionConstants.ROBOT_TO_TURRET
            .getTranslation()
            .toTranslation2d()
            .rotateBy(launchRobotPose.getRotation());

    // v_turret = v_robot + (omega x r)
    Translation2d turretFieldVelocity =
        new Translation2d(
            fieldVelocity.vxMetersPerSecond - (omega * robotToTurretField.getY()),
            fieldVelocity.vyMetersPerSecond + (omega * robotToTurretField.getX()));

    // =========================================================================
    // PHASE 3: POST-LAUNCH BALLISTIC DRIFT (Linear Drag Model)
    // The ball is in the air. It travels in a straight line relative to the
    // turret's launch velocity, slowed down exponentially by air friction.
    // =========================================================================
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d();

    Translation2d virtualTarget = target;
    double lookaheadDistance = target.getDistance(launchTurretPose.getTranslation());

    for (int i = 0; i < 20; i++) {
      // 1. Look up the time of flight for our current estimated distance
      double timeOfFlight = timeOfFlightMap.get(lookaheadDistance);

      // 2. Apply the Drag Constant to find the effective time of flight
      double effectiveTimeOfFlight =
          DRAG_CONSTANT.get() * (1.0 - Math.exp(-timeOfFlight / DRAG_CONSTANT.get()));

      // 3. Calculate how far the ball will drift sideways in the air
      Translation2d drift = turretFieldVelocity.times(effectiveTimeOfFlight);

      // 4. Shift the target exactly opposite of our drift to cancel it out
      virtualTarget = target.minus(drift);

      // 5. Update the distance (from the turret to the newly shifted virtual target)
      lookaheadDistance = virtualTarget.getDistance(launchTurretPose.getTranslation());
    }

    Logger.recordOutput(
        "LaunchCalculator/VirtualTarget", new Pose2d(virtualTarget, new Rotation2d()));
    Logger.recordOutput("LaunchCalculator/LookaheadDistance", lookaheadDistance);

    // =========================================================================
    // PHASE 4: FINAL AIMING & FEEDFORWARD
    // Aim the predicted turret at the final shifted virtual target.
    // =========================================================================
    Rotation2d turretTargetAngle =
        virtualTarget
            .minus(launchTurretPose.getTranslation())
            .getAngle()
            .minus(launchRobotPose.getRotation());

    Logger.recordOutput("LaunchCalculator/TurretTargetDegrees", turretTargetAngle.getDegrees());

    hoodAngle = hoodAngleMap.get(lookaheadDistance);

    if (lastTurretAngle == null) lastTurretAngle = turretTargetAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    turretVelocity =
        turretAngleFilter.calculate(turretTargetAngle.minus(lastTurretAngle).getRadians() / 0.02);

    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);

    lastTurretAngle = turretTargetAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new LaunchingParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            turretTargetAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelSpeedMap.get(lookaheadDistance));

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
