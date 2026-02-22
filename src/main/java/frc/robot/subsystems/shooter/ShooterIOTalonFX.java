package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  // Status Signals - Left Motor
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Temperature> leftTempCelsius;

  // Status Signals - Right Motor
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Temperature> rightTempCelsius;

  // Control objects
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl =
      new VelocityVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public ShooterIOTalonFX() {
    leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, Constants.CANIVORE_BUS);
    rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, Constants.CANIVORE_BUS);

    // Configure both motors
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.SENSOR_TO_MECHANISM_RATIO;

    // PID defaults
    controllerConfig.kP = ShooterConstants.KP;
    controllerConfig.kI = ShooterConstants.KI;
    controllerConfig.kD = ShooterConstants.KD;
    controllerConfig.kS = ShooterConstants.KS;
    controllerConfig.kV = ShooterConstants.KV;
    controllerConfig.kA = ShooterConstants.KA;

    // Apply base configuration to both motors
    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));

    // Set motor inversions
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig));

    // Apply PID config
    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(controllerConfig));
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(controllerConfig));

    // Get status signals
    leftPosition = leftMotor.getPosition();
    leftVelocity = leftMotor.getVelocity();
    leftAppliedVolts = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftTempCelsius = leftMotor.getDeviceTemp();

    rightPosition = rightMotor.getPosition();
    rightVelocity = rightMotor.getVelocity();
    rightAppliedVolts = rightMotor.getMotorVoltage();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightTempCelsius = rightMotor.getDeviceTemp();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftSupplyCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightSupplyCurrent,
        rightTempCelsius);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals and check connection status
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftSupplyCurrent, leftTempCelsius)
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTempCelsius)
            .isOK();

    // Update left motor inputs
    // inputs.leftPositionRads = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

    // Update right motor inputs
    // inputs.rightPositionRads = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftMotor.setControl(voltageControl.withOutput(leftVolts));
    rightMotor.setControl(new StrictFollower(ShooterConstants.LEFT_MOTOR_ID));
  }

  @Override
  public void stop() {
    leftMotor.setControl(neutralControl);
    rightMotor.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double rpm) {
    leftMotor.setControl(velocityControl.withVelocity(rpm / 60.0));
    rightMotor.setControl(new StrictFollower(ShooterConstants.LEFT_MOTOR_ID));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(controllerConfig));
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    controllerConfig.kS = kS;
    controllerConfig.kV = kV;
    controllerConfig.kA = kA;
    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(controllerConfig));
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(controllerConfig));
  }
}
