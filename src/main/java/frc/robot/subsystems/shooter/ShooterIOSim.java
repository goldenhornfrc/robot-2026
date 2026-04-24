package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  // 2 Kraken X44 motors driving the shared flywheel
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(2);

  private static final double MECHANISM_INERTIA = 0.00106603125;
  // 18/15 sensor-to-mechanism ratio
  private static final double GEAR_RATIO = 18.0 / 15.0;

  private final FlywheelSim shooterSim;

  // Controller tuned for Phoenix 6 Units (Rotations per Second)
  private final PIDController velocityController = new PIDController(0.45, 0.0, 0.0);
  private boolean closedLoop = false;

  private double targetRps = 0.0;
  private double appliedVolts = 0.0;

  // Default FF gains from TalonFX IO
  private double kS = 0.0;
  // kV is ignored for sim
  private double kA = 0.0;

  private double simKv = 0.0;

  private double lastTargetRps = 0.0;

  public ShooterIOSim() {
    var plant = LinearSystemId.createFlywheelSystem(GEARBOX, MECHANISM_INERTIA, GEAR_RATIO);
    shooterSim = new FlywheelSim(plant, GEARBOX, 0.0); // Assuming negligible measurement noise

    // Calculate the exact kV for the simulated motor plant to eliminate steady-state error
    // plant: dx/dt = Ax + Bu. At steady state, dx/dt = 0 -> u = (-A/B) * x
    // x is rad/s, targetRps is rotations/s. So we multiply by 2*PI to get Volts per RPS.
    simKv = -plant.getA(0, 0) / plant.getB(0, 0) * 2.0 * Math.PI;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // 1. Get WPILib native state (Radians per Second)
    double simVelRadPerSec = shooterSim.getAngularVelocityRadPerSec();

    // 2. Convert to Phoenix 6 standard units (Rotations per Second)
    double currentRps = simVelRadPerSec / (2.0 * Math.PI);

    if (closedLoop) {
      // Calculate target acceleration for kA feedforward (RPS/s)
      double targetAccelRps2 = (targetRps - lastTargetRps) / 0.02;

      // Use the mathematically exact simulation kV instead of the tuned physical kV
      // to eliminate steady-state error driven by mismatch in simulated vs real plant.
      double ff = kS * Math.signum(targetRps) + simKv * targetRps + kA * targetAccelRps2;

      // Calculate PID error using RPS
      double pidVolts = velocityController.calculate(currentRps, targetRps);

      appliedVolts = pidVolts + ff;
    }

    // Apply voltage and step the simulation forward
    shooterSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    shooterSim.update(0.02);

    // 3. Populate Inputs
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;

    // Convert back to RPM for the inputs struct
    double currentRpm = currentRps * 60.0;
    inputs.leftVelocityRpm = currentRpm;
    inputs.rightVelocityRpm = currentRpm; // Right is a strict follower, so velocity matches

    inputs.leftAppliedVolts = appliedVolts;
    inputs.rightAppliedVolts = appliedVolts;

    // Divide total simulated current by 2 to estimate per-motor draw
    double motorDrawAmps = Math.abs(shooterSim.getCurrentDrawAmps() / 2.0);

    // Visually simulate the 40A supply current limit clamp for telemetry
    double clampedCurrent = Math.min(motorDrawAmps, 40.0);

    inputs.leftSupplyCurrentAmps = clampedCurrent;
    inputs.rightSupplyCurrentAmps = clampedCurrent;

    inputs.leftTempCelsius = 25.0;
    inputs.rightTempCelsius = 25.0;

    lastTargetRps = targetRps;
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    closedLoop = false;
    // In real life, the right motor is a StrictFollower of the left ID.
    // So we apply the left requested voltage to the entire simulation.
    appliedVolts = leftVolts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void runVelocity(double rpm) {
    closedLoop = true;
    targetRps = rpm / 60.0;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    velocityController.setP(kP);
    velocityController.setI(kI);
    velocityController.setD(kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    this.kS = kS;
    // We strictly ignore the user-provided physical kV because the `LinearSystemId`
    // physics model has its own perfect theoretical simulated kV!
    // this.kV = kV;
    this.kA = kA;
  }
}
