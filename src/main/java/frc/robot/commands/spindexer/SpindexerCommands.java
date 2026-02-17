package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.spindexer.Spindexer;

public class SpindexerCommands {

  public static Command setSpindexerVoltage(double voltage, Spindexer spindexer) {
    return Commands.runEnd(
        () -> spindexer.setVoltage(voltage),
        () -> {
          spindexer.setVoltage(0);
        },
        spindexer);
  }
}
