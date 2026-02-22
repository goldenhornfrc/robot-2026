package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TrackTarget extends Command {
  private final Turret turret;
  private final DoubleSupplier goalAngleDegSupplier;
  private final DoubleSupplier robotAngleDegSupplier;
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

  private static final double MIN_ANGLE_DEG = -304.0;
  private static final double MAX_ANGLE_DEG = 75.0;

  /**
   * Tracks a field-relative target angle with the turret.
   *
   * @param turret The turret subsystem
   * @param goalAngleDegSupplier Supplier for the global field-relative target angle in degrees
   * @param robotAngleDegSupplier Supplier for the robot's current heading in degrees
   */
  public TrackTarget(
      Turret turret, DoubleSupplier goalAngleDegSupplier, DoubleSupplier robotAngleDegSupplier) {
    this.turret = turret;
    this.goalAngleDegSupplier = goalAngleDegSupplier;
    this.robotAngleDegSupplier = robotAngleDegSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (!Turret.turretCalibrationDone) {
      return;
    }

    // 1. Get raw angles (Primitive doubles prevent garbage collection overhead)
    double goalAngle = goalAngleDegSupplier.getAsDouble();
    double robotAngle = robotAngleDegSupplier.getAsDouble();
    double currentTurretAngle = turret.getTurretAngle();

    // 2. Calculate the goal angle relative to the robot chassis
    double relativeGoalDeg = goalAngle - robotAngle;

    // 3. Find the shortest path error from where the turret currently is
    // MathUtil.inputModulus wraps the error to be between -180 and 180 degrees
    double shortestPathError =
        MathUtil.inputModulus(relativeGoalDeg - currentTurretAngle, -180.0, 180.0);

    // 4. Calculate the ideal setpoint based on the shortest path
    double idealSetpoint = currentTurretAngle + shortestPathError;

    // 5. Handle the hard stops (The "Wrap" logic)
    // If the shortest path pushes us past a hard stop, we must go the long way around.
    if (idealSetpoint > MAX_ANGLE_DEG) {
      idealSetpoint -= 360.0;
    } else if (idealSetpoint < MIN_ANGLE_DEG) {
      idealSetpoint += 360.0;
    }

    // 6. Final safety clamp
    double clampedAngle = MathUtil.clamp(idealSetpoint, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

    Logger.recordOutput("Turret/TrackCommand/GoalPositionDeg", clampedAngle);

    // Apply the position to the subsystem
    turret.setTurretAngleWithFeedforward(
        clampedAngle, feedforward.calculate(-RobotState.getInstance().getDriveAngularVelocity()));
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}
