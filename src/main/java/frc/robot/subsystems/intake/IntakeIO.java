package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean motorConnected = false;
    public boolean motor2Connected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double appliedVolts2 = 0.0;
    public double supplyCurrentAmps2 = 0.0;
    public double tempCelsius2 = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runVolts(double voltage) {}

  public default void stop() {}
}
