package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeDeploy;

public class IntakeCommands {

  public static Command setIntakeVoltage(double voltage, Intake intake) {
    return Commands.runEnd(
        () -> intake.setVoltage(voltage),
        () -> {
          intake.setVoltage(0);
        },
        intake);
  }

  public static Command setIntakePivotVoltage(double voltage, IntakeDeploy intakePivot) {
    return Commands.runEnd(
        () -> intakePivot.setVoltage(voltage), () -> intakePivot.setVoltage(0), intakePivot);
  }
}
