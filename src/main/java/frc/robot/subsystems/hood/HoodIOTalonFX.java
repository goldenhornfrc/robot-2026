package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

  private TalonFXConfiguration controllerConfig = new TalonFXConfiguration();

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
        75.0, position, velocity, appliedVolts, supplyCurrent, tempCelsius);
  }

  public void configHoodTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    controllerConfig = new TalonFXConfiguration();

    controllerConfig.Slot0.kP = HoodConstants.kP;
    controllerConfig.Slot0.kI = 0;
    controllerConfig.Slot0.kD = HoodConstants.kD;

    controllerConfig.Slot0.kS = HoodConstants.kS;
    controllerConfig.Slot0.kV = HoodConstants.kV;
    controllerConfig.Slot0.kA = HoodConstants.kA;

    controllerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    controllerConfig.Feedback.FeedbackRotorOffset = 0.0;
    controllerConfig.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_SENSOR_TO_MECHANISM_RATIO;

    controllerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    controllerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    controllerConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    controllerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    controllerConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;
    controllerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    controllerConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kHoodCruiseVel;
    controllerConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.kHoodAccel;

    controllerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    controllerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    controllerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.kHoodExtendLimit / 360.0;
    controllerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.kHoodStartingPos / 360.0;

    controllerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    controllerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    controllerConfig.Audio.BeepOnConfig = true;

    tryUntilOk(5, () -> talon.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public void setHoodAngle(double angle) {
    var rotAngle = angle / 360.0;
    // hoodMotor
    //    .getConfigurator()
    //    .apply(
    //        new MotionMagicConfigs()
    //            .withMotionMagicAcceleration(HoodConstants.kHoodAccel)
    //            .withMotionMagicCruiseVelocity(HoodConstants.kHoodCruiseVel));
    hoodMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public void setHoodAngle(double angle, double cruiseVel, double acceleration) {
    var rotAngle = angle / 360;
    // hoodMotor
    //    .getConfigurator()
    //    .apply(
    //        new MotionMagicConfigs()
    //            .withMotionMagicAcceleration(acceleration)
    //            .withMotionMagicCruiseVelocity(cruiseVel));
    hoodMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public void resetHoodAngle(double angle) {
    tryUntilOk(5, () -> hoodMotor.setPosition(angle / 360.0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.Slot0.kP = kP;
    controllerConfig.Slot0.kI = kI;
    controllerConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    controllerConfig.Slot0.kS = kS;
    controllerConfig.Slot0.kV = kV;
    controllerConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(controllerConfig));
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
