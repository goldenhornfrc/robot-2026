package frc.robot.subsystems.turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public static boolean turretCalibrationDone = false;

  private final Alert motorDisconnected;
  private final Alert turretCalibrationNotReady;

  private double startTime;
  // Debouncer to require the hall sensor read true continuously for 3 seconds
  private final Debouncer turretHallStableDebouncer =
      new Debouncer(2.0, Debouncer.DebounceType.kRising);
  // PID tuning
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Turret/kP", TurretConstants.kP);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Turret/kD", TurretConstants.kD);

  // Feedforward tuning
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Turret/kS", TurretConstants.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Turret/kV", TurretConstants.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Turret/kA", TurretConstants.kA);

  public static final LoggedTunableNumber turretTargetOverride =
      new LoggedTunableNumber("Turret/TargetOverridePos", 0.0);
  private final DigitalInput turretHallSensor = new DigitalInput(9);

  private double targetAngle = 0.0;
  public static boolean wrappingAngle = false;

  public Turret(TurretIO io) {
    this.io = io;
    motorDisconnected = new Alert("Turret motor disconnected!", Alert.AlertType.kWarning);
    turretCalibrationNotReady =
        new Alert("Turret calibration has NOT been done!", Alert.AlertType.kError);
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    turretCalibrationNotReady.set(!turretCalibrationDone);
    // Update PID tuning
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);

    // Update feedforward tuning
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);

    boolean bootDone = (Timer.getFPGATimestamp() - startTime) >= 1.0;

    if (!turretCalibrationDone && bootDone && DriverStation.isDisabled()) {
      boolean hallStable = turretHallStableDebouncer.calculate(!turretHallSensor.get());
      if (hallStable) {
        turretCalibrationDone = true;
        io.resetTurretAngle(TurretConstants.kTurretResetAngle);
      }
    }
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setTurretAngle(double angle) {
    this.targetAngle = angle;
    io.setTurretAngle(angle);
  }

  public void setTurretAngleWithFeedforward(double angle, double ff) {
    this.targetAngle = angle;
    io.setTurretAngleWithFeedforward(angle, ff);
  }

  public void setTurretAngle(double angle, double cruiseVel, double acceleration) {
    this.targetAngle = angle;
    io.setTurretAngle(angle, cruiseVel, acceleration);
  }

  public double getTurretAngle() {
    return inputs.positionDegrees;
  }

  @AutoLogOutput(key = "Turret/AtGoal")
  public boolean atGoal() {
    return Math.abs(getTurretAngle() - targetAngle) <= 1.5;
  }

  @AutoLogOutput(key = "Turret/WrappingAngle")
  public boolean getWrappingAngle() {
    return wrappingAngle;
  }

  /** Sets the turret PID gains (kP, kI, kD) */
  public void setPID(double kP, double kI, double kD) {
    io.setPID(kP, kI, kD);
  }

  /** Sets the turret feedforward gains (kS, kV, kA) */
  public void setFF(double kS, double kV, double kA) {
    io.setFF(kS, kV, kA);
  }

  /* Resets the turret encoder to the given angle in degrees */
  public void resetTurretAngle(double angle) {
    io.resetTurretAngle(angle);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public Command setTurretVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Turret Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }

  public Command turretPositionTuningCommand(DoubleSupplier posSupplier) {
    return runEnd(
        () -> {
          setTurretAngle(posSupplier.getAsDouble());
        },
        () -> stop());
  }
}
