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
  private final SwerveDrivePoseEstimator swerveOdometry;
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

    swerveOdometry =
        new SwerveDrivePoseEstimator(
            kinematics,
            lastGyroRotation,
            lastWheelPositions,
            new Pose2d(),
            odometryStateStdDevs,
            new Matrix<>(VecBuilder.fill(999, 999, 999))); // No vision correction for odometry

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
    swerveOdometry.resetPosition(lastGyroRotation, lastWheelPositions, pose);
    swervePoseEstimator.resetPosition(lastGyroRotation, lastWheelPositions, pose);
  }

  /** Get the rotation of the estimated pose. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
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
    odometryPose =
        swerveOdometry.updateWithTime(
            observation.timestamp, lastGyroRotation, observation.wheelPositions);

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
    yawAngularGyroVel = Units.radiansToDegrees(yawAngularGyroVelRadPerSec);
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

  // MARK: - Type declarations

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  public record VisionObservation(
      double timestamp, Pose2d visionPose, Matrix<N3, N1> stdDevs, String source) {}
}
