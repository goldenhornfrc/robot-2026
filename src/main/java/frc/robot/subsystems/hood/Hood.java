package frc.robot.subsystems.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert motorDisconnected;

  public Hood(HoodIO io) {
    this.io = io;

    motorDisconnected = new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setHoodAngle(double angle) {
    io.setHoodAngle(angle);
  }

  public void setHoodAngle(double angle, double cruiseVel, double acceleration) {
    io.setHoodAngle(angle, cruiseVel, acceleration);
  }

  public double getHoodAngle() {
    return inputs.positionDegrees;
  }

  /* Resets the hood encoder to the given angle in degrees */
  public void resetHoodAngle(double angle) {
    io.resetHoodAngle(angle);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public Command setHoodVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Hood Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }
}
