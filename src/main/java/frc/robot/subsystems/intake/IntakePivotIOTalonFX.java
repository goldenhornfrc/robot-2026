package frc.robot.subsystems.intake;

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

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX pivotMotor = new TalonFX(25, Constants.CANIVORE_BUS);

  public IntakePivotIOTalonFX() {
    configPivotTalonFX(pivotMotor);
    pivotMotor.setPosition(IntakeConstants.intakePivotStartingPos / 360.0);
  }

  public void configPivotTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = 0;
    config.Slot0.kD = IntakeConstants.kD;
    config.Slot0.kG = IntakeConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.SensorToMechanismRatio = (40.0 / 12.0) * 7.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Voltage.PeakForwardVoltage = 8.0;
    config.Voltage.PeakReverseVoltage = -6.0;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.intakePivotCruiseVel;
    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.intakePivotAccel;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90.0 / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.intakePivotExtendLimitPos / 360.0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void setPivotAngle(double angle) {
    var rotAngle = angle / 360.0;
    pivotMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(IntakeConstants.intakePivotAccel)
                .withMotionMagicCruiseVelocity(IntakeConstants.intakePivotCruiseVel));
    pivotMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public void setPivotAngle(double angle, double cruiseVel, double acceleration) {
    var rotAngle = angle / 360;
    pivotMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicCruiseVelocity(cruiseVel));
    pivotMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public double getPivotAngle() {
    return pivotMotor.getPosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void resetPivotAngle(double angle) {
    tryUntilOk(5, () -> pivotMotor.setPosition(angle / 360.0));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {}
}
