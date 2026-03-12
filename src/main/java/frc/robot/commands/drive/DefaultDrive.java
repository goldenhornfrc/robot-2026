// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DefaultDrive extends Command {

  private static final double DEADBAND = 0.02;
  private final Drive drive;
  private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
  private boolean isFlipped = false;
  private static final double ANGLE_KP = 2.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_DEADBAND = 0.1;

  private static final double MAINTAIN_KP = 1.5;
  private static final double MAINTAIN_KD = 0.0;

  private static final double ALIGN_KP = 2.0;

  private final PIDController angleController = new PIDController(ANGLE_KP, 0.0, ANGLE_KD);
  private final PIDController maintainController = new PIDController(MAINTAIN_KP, 0.0, MAINTAIN_KD);
  private final PIDController alignController = new PIDController(ALIGN_KP, 0, 0);
  private double targetHeading = 0.0;
  private double oldTargetHeading = 0.0;
  private double targetYPos = 0.0;
  private boolean targetHeadingChanged = false;
  private double maintainTarget;

  private final RobotState robotState = RobotState.getInstance();

  // --- NEW: Dynamic Rate Limiters ---
  private final DynamicRateLimiter xLimiter = new DynamicRateLimiter();
  private final DynamicRateLimiter yLimiter = new DynamicRateLimiter();
  private final DynamicRateLimiter omegaLimiter = new DynamicRateLimiter();

  /** Creates a new DefaultDrive. */
  public DefaultDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.reset();
    angleController.setTolerance(Math.toRadians(1.5));
    maintainTarget = robotState.getEstimatedPose().getRotation().getRadians();
    maintainController.enableContinuousInput(-Math.PI, Math.PI);
    maintainController.setTolerance(Math.toRadians(1.5));
    maintainController.reset();
    alignController.reset();
  }

  @Override
  public void execute() {
    DriveState state = drive.getDriveState();
    Logger.recordOutput("DriveState", state);
    Logger.recordOutput("MaintainTarget", Math.toDegrees(maintainTarget));
    Logger.recordOutput(
        "CurrentHeading", Math.toDegrees(robotState.getEstimatedPose().getRotation().getRadians()));
    targetHeading = drive.getTargetHeading().getRadians();
    targetYPos = drive.getTargetYPos();
    targetHeadingChanged = Math.toDegrees(Math.abs(targetHeading - oldTargetHeading)) > 1.0;

    if (targetHeadingChanged) {
      angleController.reset();
    }

    // 1. Get raw intended velocity from joysticks
    Translation2d rawLinearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband and square it
    double rawOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
    rawOmega = Math.copySign(rawOmega * rawOmega, rawOmega);

    // ---------------------------------------------------------
    // NEW: Velocity and Acceleration Limiting Logic
    // ---------------------------------------------------------
    boolean isShooting = Drive.isShooting;

    // Target Velocity Multipliers (0.0 to 1.0)
    double maxLinearVelocityFactor = isShooting ? 0.45 : 1.0;
    double maxOmegaVelocityFactor = isShooting ? 0.45 : 1.0; // Slow down spins while shooting

    // Max Acceleration (Units per second. e.g., 3.0 means 0 to 100% in 0.33s)
    double linearAccelerationLimit = isShooting ? 2.0 : 4.0;
    double omegaAccelerationLimit = isShooting ? 2.0 : 2.5;

    // Apply the max velocity clamps
    double targetX = rawLinearVelocity.getX() * maxLinearVelocityFactor;
    double targetY = rawLinearVelocity.getY() * maxLinearVelocityFactor;
    double targetOmega = rawOmega * maxOmegaVelocityFactor;

    // Apply the acceleration smoothing
    double smoothedX = xLimiter.calculate(targetX, linearAccelerationLimit);
    double smoothedY = yLimiter.calculate(targetY, linearAccelerationLimit);
    double smoothedOmega = omegaLimiter.calculate(targetOmega, omegaAccelerationLimit);

    // Pack it back into a Translation2d for the rest of your logic
    Translation2d linearVelocity = new Translation2d(smoothedX, smoothedY);
    double omega = smoothedOmega;
    // ---------------------------------------------------------

    ChassisSpeeds speeds;
    switch (state) {
      case TELEOP_DRIVE:
      default:
        if (Math.abs(omegaSupplier.getAsDouble()) < 0.1) {
          maintainTarget = robotState.getEstimatedPose().getRotation().getRadians();
          Drive.setDriveState(DriveState.MAINTAIN_HEADING);
        }

        omega = MathUtil.applyDeadband(omega, ANGLE_DEADBAND);
        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());

        isFlipped = RobotContainer.getAlliance() == Alliance.Red;
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? robotState.getEstimatedPose().getRotation().plus(new Rotation2d(Math.PI))
                    : robotState.getEstimatedPose().getRotation()));
        break;

      case SNAP_HEADING:
        omega =
            angleController.calculate(
                robotState.getEstimatedPose().getRotation().getRadians(), targetHeading);

        if (Math.abs(omegaSupplier.getAsDouble()) >= 0.05) {
          Drive.setDriveState(DriveState.TELEOP_DRIVE);
        }

        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega); // PID output overrides joysticks
        isFlipped = RobotContainer.getAlliance() == Alliance.Red;

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? robotState.getEstimatedPose().getRotation().plus(new Rotation2d(Math.PI))
                    : robotState.getEstimatedPose().getRotation()));
        break;

      case TRENCH_ALIGN:
        // 1. Get the pre-set target heading (set by the trigger, NOT calculated every loop)
        targetHeading = drive.getTargetHeading().getRadians();

        // 2. Calculate rotational effort to snap to heading
        omega =
            angleController.calculate(
                robotState.getEstimatedPose().getRotation().getRadians(), targetHeading);
        // 3. Define baseline velocities (Driver controls X, but Y will be overridden if aligned)
        double trenchVx = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
        double trenchVy = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

        // 4. Safely calculate the angle error using Rotation2d.minus()
        double angleErrorDegrees =
            Math.abs(
                robotState
                    .getEstimatedPose()
                    .getRotation()
                    .minus(Rotation2d.fromRadians(targetHeading))
                    .getDegrees());

        // 5. If we are within 5 degrees, take FULL CONTROL of the Y axis
        if (angleController.atSetpoint()) {
          // Note: We don't multiply by max speed here unless your ALIGN_KP is tuned
          trenchVy = alignController.calculate(robotState.getEstimatedPose().getY(), targetYPos);
          trenchVy *= RobotContainer.getAlliance() == Alliance.Red ? -1 : 1;
          // to output a -1 to +1 percentage. If KP outputs raw meters/sec, just use it directly!
        } else {
          trenchVx = trenchVx * 0.45;
        }

        // 6. Create the overridden speeds
        speeds = new ChassisSpeeds(trenchVx, trenchVy, omega);
        isFlipped = RobotContainer.getAlliance() == Alliance.Red;

        // 7. Drive!
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? robotState.getEstimatedPose().getRotation().plus(new Rotation2d(Math.PI))
                    : robotState.getEstimatedPose().getRotation()));
        break;

      case MAINTAIN_HEADING:
        if (Math.abs(omegaSupplier.getAsDouble()) >= 0.05) {
          Drive.setDriveState(DriveState.TELEOP_DRIVE);
          break;
        }

        omega =
            maintainController.calculate(
                robotState.getEstimatedPose().getRotation().getRadians(), maintainTarget);
        omega = MathUtil.applyDeadband(omega, ANGLE_DEADBAND);

        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega); // PID output overrides joysticks
        isFlipped = RobotContainer.getAlliance() == Alliance.Red;

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? robotState.getEstimatedPose().getRotation().plus(new Rotation2d(Math.PI))
                    : robotState.getEstimatedPose().getRotation()));
        break;
    }

    oldTargetHeading = targetHeading;
  }

  @Override
  public void end(boolean interrupted) {
    Drive.setDriveState(DriveState.TELEOP_DRIVE);
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
  /** Helper class to cleanly limit acceleration of a given input. */
  public static class DynamicRateLimiter {
    private double lastValue = 0.0;

    public double calculate(double targetValue, double maxRatePerSecond) {
      double maxChangePerLoop = maxRatePerSecond * 0.02; // Assuming standard 50hz (20ms) loop
      lastValue += MathUtil.clamp(targetValue - lastValue, -maxChangePerLoop, maxChangePerLoop);
      return lastValue;
    }
  }
}
