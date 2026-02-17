// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private ShooterIO io;

  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Debouncer leftMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert leftMotorDisconnected;
  private final Alert rightMotorDisconnected;

  private double targetRpm = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;

    leftMotorDisconnected =
        new Alert("Left flywheel motor disconnected!", Alert.AlertType.kWarning);
    rightMotorDisconnected =
        new Alert("Right flywheel motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Subsystem", inputs);

    leftMotorDisconnected.set(!leftMotorConnectedDebouncer.calculate(inputs.leftMotorConnected));
    rightMotorDisconnected.set(!rightMotorConnectedDebouncer.calculate(inputs.rightMotorConnected));
  }

  public void stop() {
    io.stop();
  }

  public void setVoltage(double voltage) {
    io.runVolts(voltage, voltage);
  }

  /**
   * Set the target RPM for the shooter using velocity control with feedforward.
   *
   * @param rpm Target RPM for the shooter
   */
  public void setTargetRpm(double rpm) {
    this.targetRpm = rpm;
    io.runVelocity(rpm, 0.0);
  }

  /**
   * Set the target RPM with custom feedforward.
   *
   * @param rpm Target RPM
   * @param feedforward Feedforward voltage
   */
  public void setTargetRpmWithFeedforward(double rpm, double feedforward) {
    this.targetRpm = rpm;
    io.runVelocity(rpm, feedforward);
  }

  /**
   * Set PID gains for the shooter velocity controller.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  public void setPID(double kP, double kI, double kD) {
    io.setPID(kP, kI, kD);
  }

  /**
   * Check if the shooter is at the target RPM within tolerance.
   *
   * @return true if both motors are at target RPM within tolerance
   */
  public boolean atSetpoint() {
    return Math.abs(getVelocityRpm() - targetRpm) < ShooterConstants.RPM_TOLERANCE;
  }

  /**
   * Get the current velocity in RPM (average of both motors).
   *
   * @return Average velocity in RPM
   */
  public double getVelocityRpm() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  /**
   * Get the current target RPM.
   *
   * @return Target RPM
   */
  
  public double getTargetRpm() {
    return targetRpm;
  }

  public Command shooterVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), () -> stop())
        .withName("Shooter Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }
}
