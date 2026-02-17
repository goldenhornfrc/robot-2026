package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor = new TalonFX(15, Constants.CANIVORE_BUS);

  public HoodIOTalonFX() {
    configHoodTalonFX(hoodMotor);
    resetHoodAngle(HoodConstants.kHoodStartingPos);
  }

  public void configHoodTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kG = 0.0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.SensorToMechanismRatio = (52.0 / 8.0) * 16.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kHoodCruiseVel;
    config.MotionMagic.MotionMagicAcceleration = HoodConstants.kHoodAccel;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.kHoodStartingPos / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.kHoodExtendLimit / 360.0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public void setHoodAngle(double angle) {
    var rotAngle = angle / 360.0;
    hoodMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(HoodConstants.kHoodAccel)
                .withMotionMagicCruiseVelocity(HoodConstants.kHoodCruiseVel));
    hoodMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public void setHoodAngle(double angle, double cruiseVel, double acceleration) {
    var rotAngle = angle / 360;
    hoodMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicCruiseVelocity(cruiseVel));
    hoodMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public double getHoodAngle() {
    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void resetHoodAngle(double angle) {
    tryUntilOk(5, () -> hoodMotor.setPosition(angle / 360.0));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {}
}
