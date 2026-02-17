package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftMotorConnected = false;
    public boolean rightMotorConnected = false;
    public double leftPositionRads = 0.0;
    public double rightPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double rightVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;
    public double rightTempCelsius = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runVolts(double leftVolts, double rightVolts) {}

  public default void stop() {}

  public default void runVelocity(double rpm, double feedforward) {}

  public default void setPID(double kP, double kI, double kD) {}
}
