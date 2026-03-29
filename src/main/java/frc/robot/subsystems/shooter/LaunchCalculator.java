package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
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

  public enum DesiredAction {
    SHOOT,
    FEED
  }

  public DesiredAction desiredAction = DesiredAction.SHOOT;

  private LoggedTunableNumber DRAG_CONSTANT =
      new LoggedTunableNumber("LaunchCalculator/DragConstant", 2.15); // 1.8

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

  private static final InterpolatingDoubleTreeMap feedHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.18;
    maxDistance = 6.0;
    phaseDelay = 0.03;

    hoodAngleMap.put(1.18, 5.0);
    hoodAngleMap.put(1.7, 9.0);
    hoodAngleMap.put(2.18, 12.0);
    hoodAngleMap.put(2.7, 14.0);
    hoodAngleMap.put(3.2, 18.5);
    hoodAngleMap.put(3.7, 20.0);
    hoodAngleMap.put(4.2, 21.0);
    hoodAngleMap.put(4.6, 23.0);
    hoodAngleMap.put(4.7, 25.0);
    hoodAngleMap.put(5.3, 25.0);
    hoodAngleMap.put(5.7, 25.0);
    hoodAngleMap.put(6.0, 25.0);

    feedHoodAngleMap.put(12.0, 24.0);
    feedHoodAngleMap.put(5.2, 24.0);

    feedFlywheelSpeedMap.put(12.0, 4700.0);
    feedFlywheelSpeedMap.put(7.5, 3900.0);
    feedFlywheelSpeedMap.put(5.2, 3300.0);

    flywheelSpeedMap.put(1.18, 2900.0);
    flywheelSpeedMap.put(1.7, 3000.0);
    flywheelSpeedMap.put(2.18, 3050.0);
    flywheelSpeedMap.put(2.7, 3050.0);
    flywheelSpeedMap.put(3.2, 3150.0);
    flywheelSpeedMap.put(3.7, 3300.0);
    flywheelSpeedMap.put(4.2, 3500.0);
    flywheelSpeedMap.put(4.6, 3650.0);
    flywheelSpeedMap.put(4.7, 3700.0);
    flywheelSpeedMap.put(5.3, 3800.0);
    flywheelSpeedMap.put(5.7, 3900.0);
    flywheelSpeedMap.put(6.0, 3950.0);

    timeOfFlightMap.put(5.7, 1.25);
    timeOfFlightMap.put(5.3, 1.23);
    timeOfFlightMap.put(4.3, 1.02);
    timeOfFlightMap.put(3.3, 1.23);
    timeOfFlightMap.put(2.3, 1.14);
    timeOfFlightMap.put(1.3, 1.18);
    timeOfFlightMap.put(1.1, 1.18);
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
    Translation2d target;
    if (desiredAction == DesiredAction.FEED) {
      var outpostPos = AllianceFlipUtil.apply(FieldConstants.Outpost.centerPoint);
      if (estimatedPose.getY() >= (FieldConstants.fieldWidth / 2.0)) {
        target =
            RobotContainer.getAlliance() == Alliance.Blue
                ? new Translation2d(
                    outpostPos.getX(), FieldConstants.fieldWidth - outpostPos.getY())
                : outpostPos;
      } else {
        target =
            RobotContainer.getAlliance() == Alliance.Blue
                ? outpostPos
                : new Translation2d(
                    outpostPos.getX(), FieldConstants.fieldWidth - outpostPos.getY());
      }
    } else {
      target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d();
    }

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

    hoodAngle =
        desiredAction == DesiredAction.SHOOT
            ? hoodAngleMap.get(lookaheadDistance)
            : feedHoodAngleMap.get(lookaheadDistance);

    if (lastTurretAngle == null) lastTurretAngle = turretTargetAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    turretVelocity =
        turretAngleFilter.calculate(turretTargetAngle.minus(lastTurretAngle).getRadians() / 0.02);

    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);

    lastTurretAngle = turretTargetAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new LaunchingParameters(
            desiredAction == DesiredAction.SHOOT
                ? (lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance)
                : true,
            turretTargetAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            desiredAction == DesiredAction.SHOOT
                ? flywheelSpeedMap.get(lookaheadDistance)
                : feedFlywheelSpeedMap.get(lookaheadDistance));

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
