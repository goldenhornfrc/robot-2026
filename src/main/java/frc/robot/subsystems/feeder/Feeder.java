package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private FeederIO io;

  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;
  // Logged tunables for Talon feeder PID/FF tuning
  private static final LoggedTunableNumber feeder_kP =
      new LoggedTunableNumber("Feeder/Talon/kP", 0.5);
  private static final LoggedTunableNumber feeder_kV =
      new LoggedTunableNumber("Feeder/Talon/kV", 0.255);

  public Feeder(FeederIO io) {
    this.io = io;
    motorDisconnected = new Alert("Feeder motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], 0.0, 0.0), feeder_kP);
    LoggedTunableNumber.ifChanged(hashCode(), ffv -> io.setFF(0.0, ffv[0], 0.0), feeder_kV);
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

  public void setPID(double kP, double kI, double kD) {
    io.setPID(kP, kI, kD);
  }

  public void setFF(double kS, double kV, double kA) {
    io.setFF(kS, kV, kA);
  }

  @AutoLogOutput(key = "Feeder/Talon/VelocityRPM")
  public double getVelocityRpm() {
    return inputs.velocityRPM;
  }

  public Command setFeederVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Feeder Voltage Command");
  }

  public Command setFeederRPM(DoubleSupplier rpmSupplier) {
    return runEnd(() -> runVelocity(rpmSupplier.getAsDouble()), this::stop);
  }

  public Command runFeederVelocityCommand(DoubleSupplier velocitySupplier) {
    return runEnd(() -> runVelocity(velocitySupplier.getAsDouble()), this::stop)
        .withName("Feeder Velocity Command");
  }
}
