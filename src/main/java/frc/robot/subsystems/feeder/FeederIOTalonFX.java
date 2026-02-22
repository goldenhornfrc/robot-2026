package frc.robot.subsystems.feeder;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public FeederIOTalonFX() {
    feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID, Constants.CANIVORE_BUS);

    configFeederTalonFX(feederMotor);

    // Get StatusSignals
    appliedVolts = feederMotor.getMotorVoltage();
    supplyCurrent = feederMotor.getSupplyCurrent();
    tempCelsius = feederMotor.getDeviceTemp();

    // Set update frequency for all signals (100 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, supplyCurrent, tempCelsius);
  }

  public void configFeederTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kG = 0.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = FeederConstants.FEEDER_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = FeederConstants.FEEDER_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
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
        BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, tempCelsius).isOK();

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }
}
