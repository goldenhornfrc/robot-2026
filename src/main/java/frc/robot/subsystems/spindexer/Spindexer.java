// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private SpindexerIO io;

  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer Subsystem", inputs);
  }

  /**
   * Set the voltage for the spindexer motor.
   *
   * @param voltage Voltage to apply (-12 to 12 V)
   */
  public void setVoltage(double voltage) {
    io.runVolts(voltage);
  }

  /** Stop the spindexer motor. */
  public void stop() {
    io.stop();
  }
}
