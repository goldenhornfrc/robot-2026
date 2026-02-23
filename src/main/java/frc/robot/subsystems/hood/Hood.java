package frc.robot.subsystems.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double goalAngle = 0.0;
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Hood/kP", HoodConstants.kP);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", 0.0);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Hood/kD", HoodConstants.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Hood/kS", HoodConstants.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Hood/kV", HoodConstants.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Hood/kA", HoodConstants.kA);

  public static final LoggedTunableNumber targetAngleOverride =
      new LoggedTunableNumber("Hood/OverrideTargetAngle", 0.0);

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

    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(hashCode(), kSVA -> setFF(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setHoodAngle(double angle) {
    this.goalAngle = angle;
    io.setHoodAngle(angle);
  }

  public void setHoodAngle(double angle, double cruiseVel, double acceleration) {
    this.goalAngle = angle;
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

  /**
   * Set PID gains for the hood position controller.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  public void setPID(double kP, double kI, double kD) {
    io.setPID(kP, kI, kD);
  }

  /**
   * Set feedforward gains for the hood position controller.
   *
   * @param kS Static friction
   * @param kV Velocity feedforward
   * @param kA Acceleration feedforward
   */
  public void setFF(double kS, double kV, double kA) {
    io.setFF(kS, kV, kA);
  }

  public Command setHoodVoltageCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> setVoltage(voltageSupplier.getAsDouble()), this::stop)
        .withName("Hood Voltage Command (" + voltageSupplier.getAsDouble() + "V)");
  }

  public Command hoodPositionTuningCommand(DoubleSupplier posSupplier) {
    return runEnd(
        () -> {
          setHoodAngle(posSupplier.getAsDouble());
        },
        () -> setHoodAngle(0));
  }

  public boolean atGoal() {
    return Math.abs(getHoodAngle() - goalAngle) <= 0.5;
  }
}
