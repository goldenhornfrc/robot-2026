package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  // 2 Kraken X44 motors driving the shared flywheel
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(2);

  private static final double MECHANISM_INERTIA = 0.00696603125;
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
  private double kV = 0.115;
  private double kA = 0.0;

  private double lastTargetRps = 0.0;

  public ShooterIOSim() {
    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(GEARBOX, MECHANISM_INERTIA, GEAR_RATIO),
            GEARBOX,
            0.0); // Assuming negligible measurement noise
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

      // Phoenix 6's VelocityVoltage mode calculates feedforward based on the *target* velocity
      double ff = kS * Math.signum(targetRps) + kV * targetRps + kA * targetAccelRps2;

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
    this.kV = kV;
    this.kA = kA;
  }
}
