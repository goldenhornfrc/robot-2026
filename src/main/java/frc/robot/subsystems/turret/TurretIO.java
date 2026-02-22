package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurretAngle(double angle) {}

  public default void setTurretAngleWithFeedforward(double angle, double ff) {}

  public default void setTurretAngle(double angle, double cruiseVel, double acceleration) {}

  /* Sets turret encoder unit to corresponding angle in degrees */
  public default void resetTurretAngle(double angle) {}

  public default void setVoltage(double voltage) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}
}
