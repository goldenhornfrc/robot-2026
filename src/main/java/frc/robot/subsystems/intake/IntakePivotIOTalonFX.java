package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX pivotMotor;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakePivotIOTalonFX() {
    pivotMotor = new TalonFX(IntakeConstants.INTAKE_PIVOT_MOTOR_ID, Constants.CANIVORE_BUS);

    configPivotTalonFX(pivotMotor);
    pivotMotor.setPosition(IntakeConstants.intakePivotStartingPos / 360.0);

    // Get StatusSignals
    position = pivotMotor.getPosition();
    velocity = pivotMotor.getVelocity();
    appliedVolts = pivotMotor.getMotorVoltage();
    supplyCurrent = pivotMotor.getSupplyCurrent();
    tempCelsius = pivotMotor.getDeviceTemp();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, supplyCurrent, tempCelsius);
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
    config.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_PIVOT_SENSOR_TO_MECHANISM_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Voltage.PeakForwardVoltage = 8.0;
    config.Voltage.PeakReverseVoltage = -6.0;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_PIVOT_STATOR_CURRENT_LIMIT;
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
  public void resetPivotAngle(double angle) {
    tryUntilOk(5, () -> pivotMotor.setPosition(angle / 360.0));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
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
