package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TrackTarget extends Command {
  private final Turret turret;
  private final DoubleSupplier robotRelativeAngleDegSupplier;
  private final DoubleSupplier turretVelocityRadPerSecSupplier;
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

  private static final double MIN_ANGLE_DEG = TurretConstants.kTurretCWLimit;
  private static final double MAX_ANGLE_DEG = TurretConstants.kTurretCCWLimit;

  /**
   * Tracks a robot-relative target angle with the turret using predictive feedforward.
   *
   * @param turret The turret subsystem
   * @param robotRelativeAngleDegSupplier Supplier for the robot-relative target angle in degrees
   * @param turretVelocityRadPerSecSupplier Supplier for the predictive feedforward velocity
   */
  public TrackTarget(
      Turret turret,
      DoubleSupplier robotRelativeAngleDegSupplier,
      DoubleSupplier turretVelocityRadPerSecSupplier) {
    this.turret = turret;
    this.robotRelativeAngleDegSupplier = robotRelativeAngleDegSupplier;
    this.turretVelocityRadPerSecSupplier = turretVelocityRadPerSecSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (!Turret.turretCalibrationDone) {
      return;
    }

    // 1. Get the pre-calculated robot-relative goal directly
    double relativeGoalDeg = robotRelativeAngleDegSupplier.getAsDouble();
    double currentTurretAngle = turret.getTurretAngle();

    Drive.isShooting = true;
    // 2. Find the shortest path error from where the turret currently is
    double shortestPathError =
        MathUtil.inputModulus(relativeGoalDeg - currentTurretAngle, -180.0, 180.0);

    // 3. Calculate the ideal setpoint based on the shortest path
    double idealSetpoint = currentTurretAngle + shortestPathError;

    // 4. Handle the hard stops
    if (idealSetpoint > MAX_ANGLE_DEG) {
      idealSetpoint -= 360.0;
    } else if (idealSetpoint < MIN_ANGLE_DEG) {
      idealSetpoint += 360.0;
    }

    // 5. Final safety clamp
    double clampedAngle = MathUtil.clamp(idealSetpoint, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

    double errorToSetpoint = Math.abs(clampedAngle - currentTurretAngle);

    // If the error is > 180, it means the hard stop logic shifted the setpoint 360 degrees.
    if (errorToSetpoint > 180.0) {
      Turret.wrappingAngle = true; // Wrap initiated
    }
    // Keep it true until the turret physically swings around and gets close to the target
    else if (errorToSetpoint < 5.0) {
      Turret.wrappingAngle = false; // Wrap finished
    }

    Logger.recordOutput("Turret/TrackCommand/GoalPositionDeg", clampedAngle);

    double velocityDegPerSec = Math.toDegrees(turretVelocityRadPerSecSupplier.getAsDouble());
    turret.setTurretAngleWithFeedforward(clampedAngle, feedforward.calculate(velocityDegPerSec));
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    Drive.isShooting = false;
  }
}
