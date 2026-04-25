package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
  @AutoLog
  public static class IntakeDeployIOInputs {
    public boolean motorConnected = false;
    public double positionRotations = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}

  public default void setDeployPos(double rotations) {}

  public default void setDeployPos(double rotations, double cruiseVel, double acceleration) {}

  public default void resetDeployPos(double rotations) {}

  public default void setVoltage(double voltage) {}
}
