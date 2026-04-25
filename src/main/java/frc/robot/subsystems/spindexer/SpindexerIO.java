package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double velocityRPM = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void runVolts(double voltage) {}

  public default void runVelocity(double velocityRPM) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void stop() {}
}
