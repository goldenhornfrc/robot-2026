package frc.robot.subsystems.intake;

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

public class IntakeDeployIOTalonFX implements IntakeDeployIO {
  private final TalonFX deployMotor;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakeDeployIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, Constants.CANIVORE_BUS);

    configDeployTalonFX(deployMotor);
    // deployMotor.setPosition(IntakeConstants.intakeDeployStartingPos / 360.0);

    // Get StatusSignals
    position = deployMotor.getPosition();
    velocity = deployMotor.getVelocity();
    appliedVolts = deployMotor.getMotorVoltage();
    supplyCurrent = deployMotor.getSupplyCurrent();
    tempCelsius = deployMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, tempCelsius);
  }

  public void configDeployTalonFX(TalonFX talon) {
    ;
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = 0;
    config.Slot0.kD = IntakeConstants.kD;
    config.Slot0.kG = 0.0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.SensorToMechanismRatio =
        IntakeConstants.INTAKE_DEPLOY_SENSOR_TO_MECHANISM_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Voltage.PeakForwardVoltage = 8.0;
    config.Voltage.PeakReverseVoltage = -8.0;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_DEPLOY_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_DEPLOY_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.intakeDeployCruiseVel;
    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.intakeDeployAccel;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; // TODO: tune and enable
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.intakeDeployExtendLimitPos;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.intakeDeployStartingPos;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setVoltage(double voltage) {
    deployMotor.setVoltage(voltage);
  }

  @Override
  public void setDeployPos(double rotations) {
    deployMotor.setControl(new MotionMagicVoltage(rotations).withSlot(0));
  }

  @Override
  public void setDeployPos(double rotations, double cruiseVel, double acceleration) {
    deployMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicCruiseVelocity(cruiseVel));
    deployMotor.setControl(new MotionMagicVoltage(rotations).withSlot(0));
  }

  @Override
  public void resetDeployPos(double rotations) {
    tryUntilOk(5, () -> deployMotor.setPosition(rotations));
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    // Refresh all signals and check connection status
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, tempCelsius)
            .isOK();

    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotationsPerSecond = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }
}
