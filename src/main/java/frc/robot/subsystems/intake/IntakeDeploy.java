package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private IntakeDeployIO io;

  private IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert motorDisconnected;

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
    io.resetDeployPos(IntakeConstants.intakeDeployStartingPos);
    motorDisconnected = new Alert("Intake Deploy motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPivotPos(double rotations) {
    io.setDeployPos(rotations);
  }

  public void setPivotPos(double rotations, double cruiseVel, double acceleration) {
    io.setDeployPos(rotations, cruiseVel, acceleration);
  }

  public double getPivotPos() {
    return inputs.positionRotations;
  }

  public void resetPos(double rotations) {
    io.resetDeployPos(rotations);
  }
}
