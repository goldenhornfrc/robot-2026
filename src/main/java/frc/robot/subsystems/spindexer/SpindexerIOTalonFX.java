package frc.robot.subsystems.spindexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX motor;

  // Status Signals
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<AngularVelocity> velocity;

  // Control objects
  private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityControl =
      new VelocityVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public SpindexerIOTalonFX() {
    motor = new TalonFX(SpindexerConstants.MOTOR_ID, Constants.CANIVORE_BUS);

    // Configure motor
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = SpindexerConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = SpindexerConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = SpindexerConstants.SENSOR_TO_MECHANISM_RATIO;

    // PID and FF configuration
    config.Slot0.kP = SpindexerConstants.KP;
    config.Slot0.kI = SpindexerConstants.KI;
    config.Slot0.kD = SpindexerConstants.KD;
    config.Slot0.kS = SpindexerConstants.KS;
    config.Slot0.kV = SpindexerConstants.KV;
    config.Slot0.kA = SpindexerConstants.KA;

    // Apply configuration
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    tempCelsius = motor.getDeviceTemp();
    velocity = motor.getVelocity();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVolts, supplyCurrent, tempCelsius, velocity);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // Refresh all signals and check connection status
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, tempCelsius, velocity).isOK();

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.velocityRPM = velocity.getValueAsDouble() * 60.0;
  }

  @Override
  public void runVolts(double voltage) {
    motor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void runVelocity(double velocityRPM) {
    motor.setControl(velocityControl.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    motor.setControl(neutralControl);
  }
}
