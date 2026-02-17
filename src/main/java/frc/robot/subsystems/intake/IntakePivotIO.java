package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean motorConnected = false;
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPivotAngle(double angle) {}

  public default void setPivotAngle(double angle, double cruiseVel, double acceleration) {}

  /* Sets pivot encoder unit to corresponding angle in degrees */
  public default void resetPivotAngle(double angle) {}

  public default void setVoltage(double voltage) {}
}
