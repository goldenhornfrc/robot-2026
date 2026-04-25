package frc.robot.subsystems.feeder;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotor;

  // Status Signals
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<AngularVelocity> velocityRPM;
  private com.ctre.phoenix6.configs.Slot0Configs controllerConfig =
      new com.ctre.phoenix6.configs.Slot0Configs();

  public FeederIOTalonFX() {
    feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID, Constants.CANIVORE_BUS);

    configFeederTalonFX(feederMotor);

    // Get StatusSignals
    appliedVolts = feederMotor.getMotorVoltage();
    supplyCurrent = feederMotor.getSupplyCurrent();
    tempCelsius = feederMotor.getDeviceTemp();
    velocityRPM = feederMotor.getVelocity();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, supplyCurrent, tempCelsius);
  }

  public void configFeederTalonFX(TalonFX talon) {

    controllerConfig = new com.ctre.phoenix6.configs.Slot0Configs();

    controllerConfig.kP = 0.5;
    controllerConfig.kI = 0;
    controllerConfig.kD = 0;
    controllerConfig.kV = 0.255;

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.Slot0 = controllerConfig;

    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    baseConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    baseConfig.Feedback.SensorToMechanismRatio = 2.0;

    baseConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.FEEDER_SUPPLY_CURRENT_LIMIT;
    baseConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    baseConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.FEEDER_STATOR_CURRENT_LIMIT;
    baseConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> talon.getConfigurator().apply(baseConfig));
  }

  @Override
  public void runVolts(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    feederMotor.setVoltage(0.0);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Refresh all signals and check connection status
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, tempCelsius, velocityRPM).isOK();

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.velocityRPM = velocityRPM.getValueAsDouble() * 60.0;
  }

  @Override
  public void runVelocity(double velocityRPM) {
    feederMotor.setControl(new VelocityVoltage(velocityRPM / 60.0).withEnableFOC(true));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    tryUntilOk(5, () -> feederMotor.getConfigurator().apply(controllerConfig));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    controllerConfig.kS = kS;
    controllerConfig.kV = kV;
    controllerConfig.kA = kA;
    tryUntilOk(5, () -> feederMotor.getConfigurator().apply(controllerConfig));
  }
}
