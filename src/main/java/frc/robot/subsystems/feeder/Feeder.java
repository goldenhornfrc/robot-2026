// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private FeederIO io;

  private FeederIOInputs inputs;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }
}
