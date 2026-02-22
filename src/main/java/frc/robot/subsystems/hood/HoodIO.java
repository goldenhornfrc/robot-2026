package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = false;
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodAngle(double angle) {}

  public default void setHoodAngle(double angle, double cruiseVel, double acceleration) {}

  /* Sets hood encoder unit to corresponding angle in degrees */
  public default void resetHoodAngle(double angle) {}

  public default void setVoltage(double voltage) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}
}
