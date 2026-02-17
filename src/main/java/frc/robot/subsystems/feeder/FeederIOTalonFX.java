package frc.robot.subsystems.feeder;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX feederMotor = new TalonFX(22, Constants.CANIVORE_BUS);

  public FeederIOTalonFX() {
    configFeederTalonFX(feederMotor);
  }

  public void configFeederTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kG = 0.0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {}
}
