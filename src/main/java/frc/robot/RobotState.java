// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Bounds;
import java.util.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  // Constants
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));
  private static final Matrix<N3, N1> visionDefaultStdDevs =
      new Matrix<>(VecBuilder.fill(0.5, 0.5, 0.9)); // Default, overridden by vision observations

  // MARK: - Class fields

  // Pose estimation fields
  @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  // Odometry & Estimation fields
  private final SwerveDriveKinematics kinematics;
  // private final SwerveDrivePoseEstimator swerveOdometry;
  private final SwerveDrivePoseEstimator swervePoseEstimator;

  // Store last inputs for reset and delta calculations
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroRotation = Rotation2d.kZero;
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  private double yawAngularGyroVel = 0.0;

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(
                TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(
                TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(
                TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));

    /*
        swerveOdometry =
            new SwerveDrivePoseEstimator(
                kinematics,
                lastGyroRotation,
                lastWheelPositions,
                new Pose2d(),
                odometryStateStdDevs,
                new Matrix<>(VecBuilder.fill(999, 999, 999))); // No vision correction for odometry
    */
    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            lastGyroRotation,
            lastWheelPositions,
            new Pose2d(),
            odometryStateStdDevs,
            visionDefaultStdDevs);
  }

  // MARK: - Drive & vision methods

  /** Reset the pose estimate and odometry pose to the given pose. */
  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    odometryPose = pose;

    // Reset both the pure odometry and the estimator
    // swerveOdometry.resetPosition(lastGyroRotation, lastWheelPositions, pose);
    swervePoseEstimator.resetPosition(lastGyroRotation, lastWheelPositions, pose);
  }

  /** Get the rotation of the estimated pose. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  /** Gets the continuous, un-reset raw heading tracked mainly for internal odometry. */
  public Rotation2d getRawGyroRotation() {
    return lastGyroRotation;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  @AutoLogOutput(key = "RobotState/RobotVelocity")
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  /** Adds a new odometry sample from the drive subsystem. */
  public void addOdometryObservation(OdometryObservation observation) {
    // If gyro is present, use it.
    // If not, calculate the rotation delta from wheel modules and apply to the last known gyro
    // rotation.
    if (observation.gyroAngle.isPresent()) {
      lastGyroRotation = observation.gyroAngle.get();
    } else {
      Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions);
      lastGyroRotation = lastGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    lastWheelPositions = observation.wheelPositions;

    // Update pure odometry
    /*
    odometryPose =
        swerveOdometry.updateWithTime(
            observation.timestamp, lastGyroRotation, observation.wheelPositions);
    */
    // Update pose estimator
    estimatedPose =
        swervePoseEstimator.updateWithTime(
            observation.timestamp, lastGyroRotation, observation.wheelPositions);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Adds a new vision pose observation from the vision subsystem. */
  public void addVisionObservation(VisionObservation observation) {
    Logger.recordOutput(
        "RobotState/VisionObservation/" + observation.source, observation.visionPose);
    swervePoseEstimator.addVisionMeasurement(
        observation.visionPose, observation.timestamp, observation.stdDevs);
    // Update estimated pose to current result
    estimatedPose = swervePoseEstimator.getEstimatedPosition();
  }

  public void addDriveSpeeds(ChassisSpeeds speeds, double yawAngularGyroVelRadPerSec) {
    robotVelocity = speeds;
    if (Robot.isSimulation()) {
      yawAngularGyroVel = Units.radiansToDegrees(speeds.omegaRadiansPerSecond);
    } else {
      yawAngularGyroVel = Units.radiansToDegrees(yawAngularGyroVelRadPerSec);
    }
  }

  @AutoLogOutput(key = "RobotState/TurretToFieldAngle")
  public Rotation2d getTurretToFieldAngle(Rotation2d turretAngle) {
    return getRotation().plus(turretAngle);
  }

  @AutoLogOutput(key = "RobotState/AngularVelocity")
  /** Degrees per sec */
  public double getDriveAngularVelocity() {
    return yawAngularGyroVel;
  }

  /**
   * Predicts if the robot will be within the specified bounds after a given lookahead time.
   *
   * @param bounds The boundaries to check against.
   * @param dtSeconds The lookahead time in seconds.
   * @return True if the predicted position is within the bounds.
   */
  public boolean isPredictedToBeInBounds(Bounds bounds, double dtSeconds) {
    // 1. Get current robot-relative velocity
    ChassisSpeeds velocity = getRobotVelocity();

    // 2. Calculate the twist (change in pose) over the lookahead time
    Twist2d movementTwist =
        new Twist2d(
            velocity.vxMetersPerSecond * dtSeconds,
            velocity.vyMetersPerSecond * dtSeconds,
            velocity.omegaRadiansPerSecond * dtSeconds);

    // 3. Apply the twist to the current estimated pose to get the predicted future pose
    Pose2d predictedPose = getEstimatedPose().exp(movementTwist);

    // 4. Check if the predicted translation is within the bounds
    return bounds.contains(predictedPose.getTranslation());
  }

  /**
   * Sweeps the predicted path of the robot to see if it will pass through the bounds at ANY POINT
   * between now and the max lookahead time. Prevents high-speed "tunneling".
   */
  public boolean willPassThroughBounds(Bounds bounds, double maxLookaheadSeconds) {
    ChassisSpeeds velocity = getRobotVelocity();
    Pose2d currentPose = getEstimatedPose();

    // Step size for checking (0.1 seconds)
    // At a max FRC speed of ~5.5 m/s, 0.1s = 0.55 meters of travel per step.
    // Because the trench is > 1.1 meters deep, it is physically impossible
    // for the robot to jump over the trench in a single 0.55m step!
    double stepSize = 0.25;

    for (double t = 0; t <= maxLookaheadSeconds; t += stepSize) {
      Twist2d movementTwist =
          new Twist2d(
              velocity.vxMetersPerSecond * t,
              velocity.vyMetersPerSecond * t,
              velocity.omegaRadiansPerSecond * t);

      Pose2d predictedPose = currentPose.exp(movementTwist);

      if (bounds.contains(predictedPose.getTranslation())) {
        return true; // We found a collision along the projected path!
      }
    }

    // Do one final check exactly at maxLookaheadSeconds just in case
    // maxLookaheadSeconds isn't perfectly divisible by stepSize.
    Twist2d finalTwist =
        new Twist2d(
            velocity.vxMetersPerSecond * maxLookaheadSeconds,
            velocity.vyMetersPerSecond * maxLookaheadSeconds,
            velocity.omegaRadiansPerSecond * maxLookaheadSeconds);
    return bounds.contains(currentPose.exp(finalTwist).getTranslation());
  }

  // MARK: - Type declarations

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  public record VisionObservation(
      double timestamp, Pose2d visionPose, Matrix<N3, N1> stdDevs, String source) {}
}
