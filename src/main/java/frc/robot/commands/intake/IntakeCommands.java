package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;

public class IntakeCommands {

  public static Command setIntakeVoltage(double voltage, Intake intake) {
    return Commands.runEnd(
        () -> intake.setVoltage(voltage),
        () -> {
          intake.setVoltage(0);
        },
        intake);
  }

  public static Command setIntakePivotVoltage(double voltage, IntakePivot intakePivot) {
    return Commands.runEnd(
        () -> intakePivot.setVoltage(voltage), () -> intakePivot.setVoltage(0), intakePivot);
  }
}
