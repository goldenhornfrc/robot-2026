package frc.robot.subsystems.hood;

public interface HoodIO {
  public static class HoodIOInputs {}

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodAngle(double angle) {}

  public default void setHoodAngle(double angle, double cruiseVel, double acceleration) {}

  public default double getHoodAngle() {
    return 0.0;
  }
  /* Sets hood encoder unit to corresponding angle in degrees */
  public default void resetHoodAngle(double angle) {}

  public default void setVoltage(double voltage) {}
}
