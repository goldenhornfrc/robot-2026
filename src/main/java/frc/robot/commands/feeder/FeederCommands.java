package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;

public class FeederCommands {

  public static Command setFeederVoltage(double voltage, Feeder feeder) {
    return Commands.runEnd(
        () -> feeder.setVoltage(voltage),
        () -> {
          feeder.setVoltage(0);
        },
        feeder);
  }
}
