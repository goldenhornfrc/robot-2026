package frc.robot.subsystems.feeder;

public interface FeederIO {

  public static class FeederIOInputs {}

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
