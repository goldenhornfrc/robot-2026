package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private IntakePivotIO io;

  private IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert motorDisconnected;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    io.resetPivotAngle(IntakeConstants.intakePivotStartingPos);
    motorDisconnected = new Alert("Intake pivot motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
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

  public double getPivotAngle() {
    return inputs.positionDegrees;
  }

  public void resetPivotAngle(double angle) {
    io.resetPivotAngle(angle);
  }
}
