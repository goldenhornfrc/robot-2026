// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeDeploy;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakeDeployPos extends Command {
  private final IntakeDeploy intakeDeploy;

  private final double rotations;
  private final double cruiseVel, acceleration;
  private boolean shouldHold = false;

  public SetIntakeDeployPos(IntakeDeploy intakeDeploy, double rotations, boolean shouldHold) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeDeploy = intakeDeploy;
    this.rotations = rotations;
    this.shouldHold = shouldHold;
    cruiseVel = 0.0;
    acceleration = 0.0;
    addRequirements(intakeDeploy);
  }

  public SetIntakeDeployPos(
      IntakeDeploy intakeDeploy,
      double rotations,
      double cruiseVel,
      double acceleration,
      boolean shouldHold) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeDeploy = intakeDeploy;
    this.rotations = rotations;
    this.cruiseVel = cruiseVel;
    this.acceleration = acceleration;
    this.shouldHold = shouldHold;
    addRequirements(intakeDeploy);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (cruiseVel != 0 && acceleration != 0) {
      intakeDeploy.setPivotPos(rotations, cruiseVel, acceleration);
    } else {
      intakeDeploy.setPivotPos(rotations);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!shouldHold) {
      intakeDeploy.setVoltage(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(intakeDeploy.getPivotPos() - rotations)
        <= IntakeConstants.kIntakeDeployAllowableErrorRotations;
  }
}
