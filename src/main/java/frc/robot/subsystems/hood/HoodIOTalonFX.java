package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, Constants.CANIVORE_BUS);

    configHoodTalonFX(hoodMotor);
    resetHoodAngle(HoodConstants.kHoodStartingPos);

    // Get StatusSignals
    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    appliedVolts = hoodMotor.getMotorVoltage();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    tempCelsius = hoodMotor.getDeviceTemp();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, supplyCurrent, tempCelsius);
  }

  public void configHoodTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = HoodConstants.kP;
    config.Slot0.kI = 0;
    config.Slot0.kD = HoodConstants.kD;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_SENSOR_TO_MECHANISM_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;
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
  public void resetHoodAngle(double angle) {
    tryUntilOk(5, () -> hoodMotor.setPosition(angle / 360.0));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Refresh all signals and check connection status
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, tempCelsius)
            .isOK();

    inputs.positionDegrees = position.getValueAsDouble() * 360.0;
    inputs.velocityDegreesPerSecond = velocity.getValueAsDouble() * 360.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }
}
