// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakePivotIO.IntakePivotIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private IntakePivotIO io;

  private IntakePivotIOInputs inputs;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  public void setPivotAngle(double angle, double cruiseVel, double acceleration) {
    io.setPivotAngle(angle, cruiseVel, acceleration);
  }

  @AutoLogOutput(key = "IntakePivot/Angle")
  public double getPivotAngle() {
    return io.getPivotAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }
}
