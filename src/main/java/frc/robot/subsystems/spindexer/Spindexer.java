// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private SpindexerIO io;

  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert motorDisconnected;

  // Logged tunables for Talon spindexer PID/FF tuning
  private static final LoggedTunableNumber spindexer_kP =
      new LoggedTunableNumber("Spindexer/Talon/kP", SpindexerConstants.KP);
  private static final LoggedTunableNumber spindexer_kV =
      new LoggedTunableNumber("Spindexer/Talon/kV", SpindexerConstants.KV);

  public Spindexer(SpindexerIO io) {
    this.io = io;

    motorDisconnected = new Alert("Spindexer motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer Subsystem", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], 0.0, 0.0), spindexer_kP);
    LoggedTunableNumber.ifChanged(hashCode(), ffv -> io.setFF(0.0, ffv[0], 0.0), spindexer_kV);
  }

  /**
   * Set the voltage for the spindexer motor.
   *
   * @param voltage Voltage to apply (-12 to 12 V)
   */
  public void setVoltage(double voltage) {
    io.runVolts(voltage);
  }

  public void runVelocity(double velocityRPM) {
    io.runVelocity(velocityRPM);
  }

  public void setPID(double kP, double kI, double kD) {
    io.setPID(kP, kI, kD);
  }

  public void setFF(double kS, double kV, double kA) {
    io.setFF(kS, kV, kA);
  }

  @AutoLogOutput(key = "Spindexer/Talon/VelocityRPM")
  public double getVelocityRpm() {
    return inputs.velocityRPM;
  }

  /** Stop the spindexer motor. */
  public void stop() {
    io.stop();
  }

  public Command setSpindexerVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Spindexer Voltage Command");
  }

  public Command runSpindexerVelocityCommand(DoubleSupplier velocitySupplier) {
    return runEnd(() -> runVelocity(velocitySupplier.getAsDouble()), this::stop)
        .withName("Spindexer Velocity Command");
  }
}
