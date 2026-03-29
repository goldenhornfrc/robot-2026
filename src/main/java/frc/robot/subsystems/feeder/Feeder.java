package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  private final SparkFlex feederneo =
      new SparkFlex(12, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

  private final SparkClosedLoopController closedLoopController =
      feederneo.getClosedLoopController();

  private final Alert motorDisconnected;

  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Feeder/Neo/kV", 0.0021);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Feeder/Neo/kP", 0.00001);

  // Logged tunables for Talon feeder PID/FF tuning
  private static final LoggedTunableNumber feeder_kP =
      new LoggedTunableNumber("Feeder/Talon/kP", 2.0);
  private static final LoggedTunableNumber feeder_kV =
      new LoggedTunableNumber("Feeder/Talon/kV", 0.5);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, kV.get(), 0);

  public Feeder(FeederIO io) {
    this.io = io;
    motorDisconnected = new Alert("Feeder motor disconnected!", Alert.AlertType.kWarning);
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.pid(kP.get(), 0, 0.0);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    feederneo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    LoggedTunableNumber.ifChanged(hashCode(), feedforward -> ff.setKv(feedforward[0]), kV);
    LoggedTunableNumber.ifChanged(hashCode(), param -> setNEOPID(param[0]), kP);
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], 0.0, 0.0), feeder_kP);
    LoggedTunableNumber.ifChanged(hashCode(), ffv -> io.setFF(0.0, ffv[0], 0.0), feeder_kV);
  }

  public void setFeederNeoVelocity(double rpm) {
    double ffValue = ff.calculate(rpm);
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffValue);
  }

  public void setVoltage(double voltage) {
    io.runVolts(voltage);
    feederneo.setVoltage(voltage / 1.5);
  }

  public void setVoltageNew(double voltage, double voltageNeo) {
    io.runVolts(voltage);
    feederneo.setVoltage(voltageNeo);
  }

  /** Stop the feeder motor. */
  public void stop() {
    io.stop();
    feederneo.setVoltage(0);
  }

  public void runVelocity(double velocityRPM) {
    setFeederNeoVelocity(velocityRPM);
    io.runVelocity(velocityRPM);
  }

  private void setNEOPID(double kP) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.pid(kP, 0, 0.0);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    feederneo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

  @AutoLogOutput(key = "Feeder/Neo/VelocityRPM")
  public double getNeoVelocityRpm() {
    return feederneo.getEncoder().getVelocity();
  }

  public Command setFeederVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Feeder Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }

  public Command setFeederRPM(DoubleSupplier rpmSupplier) {
    return runEnd(() -> runVelocity(rpmSupplier.getAsDouble()), this::stop);
  }

  public Command setFeederVoltageCommandNew(
      DoubleSupplier voltageSupplier, DoubleSupplier neoVDoubleSupplier) {
    return runEnd(
            () -> setVoltageNew(voltageSupplier.getAsDouble(), neoVDoubleSupplier.getAsDouble()),
            this::stop)
        .withName("Feeder Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }

  public Command runFeederVelocityCommand(DoubleSupplier velocitySupplier) {
    return runEnd(() -> runVelocity(velocitySupplier.getAsDouble()), this::stop)
        .withName("Feeder Velocity Command (" + velocitySupplier.getAsDouble() + "RPM)");
  }
}
