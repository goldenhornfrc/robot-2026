package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private FeederIO io;

  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert motorDisconnected;

  public Feeder(FeederIO io) {
    this.io = io;

    motorDisconnected = new Alert("Feeder motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
  }

  public void setVoltage(double voltage) {
    io.runVolts(voltage);
  }

  /** Stop the feeder motor. */
  public void stop() {
    io.stop();
  }

  public void runVelocity(double velocityRPM) {
    io.runVelocity(velocityRPM);
  }

  public Command setFeederVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Feeder Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }

  public Command runFeederVelocityCommand(DoubleSupplier velocitySupplier) {
    return runEnd(() -> runVelocity(velocitySupplier.getAsDouble()), this::stop)
        .withName("Feeder Velocity Command (" + velocitySupplier.getAsDouble() + "RPM)");
  }
}
