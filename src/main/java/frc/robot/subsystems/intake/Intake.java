// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Set the intake motor voltage.
   *
   * @param voltage Voltage in volts, [-12, 12]
   */
  public void setVoltage(double voltage) {
    io.runVolts(voltage);
  }

  /** Stop the intake motor. */
  public void stop() {
    io.stop();
  }

  public Command runIntakeCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop).withName("Intake Command ("+voltageSupplier.getAsDouble()+"V)");
  }
}
