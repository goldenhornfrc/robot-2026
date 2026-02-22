package frc.robot.subsystems.turret;

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

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private TalonFXConfiguration controllerConfig = new TalonFXConfiguration();

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID, Constants.CANIVORE_BUS);

    configTurretTalonFX(turretMotor);
    // resetTurretAngle(TurretConstants.kTurretStartingPos);

    // Get StatusSignals
    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVolts = turretMotor.getMotorVoltage();
    supplyCurrent = turretMotor.getSupplyCurrent();
    tempCelsius = turretMotor.getDeviceTemp();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, supplyCurrent, tempCelsius);
  }

  public void configTurretTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    controllerConfig = new TalonFXConfiguration();

    controllerConfig.Slot0.kP = TurretConstants.kP;
    controllerConfig.Slot0.kI = 0;
    controllerConfig.Slot0.kD = TurretConstants.kD;

    controllerConfig.Slot0.kS = TurretConstants.kS;
    controllerConfig.Slot0.kV = 0.0; // TurretConstants.kV;
    controllerConfig.Slot0.kA = TurretConstants.kA;

    controllerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    controllerConfig.Feedback.FeedbackRotorOffset = 0.0;
    controllerConfig.Feedback.SensorToMechanismRatio =
        TurretConstants.TURRET_SENSOR_TO_MECHANISM_RATIO;

    controllerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    controllerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    controllerConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.TURRET_SUPPLY_CURRENT_LIMIT;
    controllerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    controllerConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.TURRET_STATOR_CURRENT_LIMIT;
    controllerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    controllerConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretCruiseVel;
    controllerConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretAccel;

    controllerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    controllerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    controllerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TurretConstants.kTurretCCWLimit / 360.0;
    controllerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        TurretConstants.kTurretCWLimit / 360.0;

    controllerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    controllerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    tryUntilOk(5, () -> talon.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public void setTurretAngle(double angle) {
    var rotAngle = angle / 360.0;
    /*
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    turretMotor.getConfigurator().refresh(readConfig);
    if (!(readConfig.MotionMagic.MotionMagicAcceleration == TurretConstants.kTurretAccel
        && readConfig.MotionMagic.MotionMagicCruiseVelocity == TurretConstants.kTurretCruiseVel)) {
      turretMotor
          .getConfigurator()
          .apply(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(TurretConstants.kTurretAccel)
                  .withMotionMagicCruiseVelocity(TurretConstants.kTurretCruiseVel));
    }
                  */
    turretMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public void setTurretAngle(double angle, double cruiseVel, double acceleration) {
    var rotAngle = angle / 360;
    turretMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicCruiseVelocity(cruiseVel));

    turretMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0));
  }

  @Override
  public void setTurretAngleWithFeedforward(double angle, double ff) {
    var rotAngle = angle / 360.0;
    turretMotor.setControl(new MotionMagicVoltage(rotAngle).withSlot(0).withFeedForward(ff));
  }

  @Override
  public void resetTurretAngle(double angle) {
    tryUntilOk(5, () -> turretMotor.setPosition(angle / 360.0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.Slot0.kP = kP;
    controllerConfig.Slot0.kI = kI;
    controllerConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> turretMotor.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    controllerConfig.Slot0.kS = kS;
    controllerConfig.Slot0.kV = kV;
    controllerConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> turretMotor.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
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
