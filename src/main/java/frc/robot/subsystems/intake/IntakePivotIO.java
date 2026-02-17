package frc.robot.subsystems.intake;

public interface IntakePivotIO {
  public static class IntakePivotIOInputs {}

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPivotAngle(double angle) {}

  public default void setPivotAngle(double angle, double cruiseVel, double acceleration) {}

  public default double getPivotAngle() {
    return 0.0;
  }
  /* Sets pivot encoder unit to corresponding angle in degrees */
  public default void resetPivotAngle(double angle) {}

  public default void setVoltage(double voltage) {}
}
