package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.Logger;

public class AutoTuneTurret extends Command {
  private final Turret turret;
  private final Timer profileTimer = new Timer();

  // Hooke & Jeeves Search States
  private enum SearchState {
    EXPLORE_KP,
    EXPLORE_KV,
    PATTERN_MOVE
  }

  private SearchState currentState = SearchState.EXPLORE_KP;

  // Tuning Bounds (CRITICAL FOR PHYSICAL SAFETY)
  private static final double MIN_KP = 0.0;
  private static final double MAX_KP = 300.0;
  private static final double MIN_KV = 0.01;
  private static final double MAX_KV = 0.1;

  // Hooke & Jeeves Variables
  private double baseKp = 125.0; // Starting guess
  private double baseKv = 0.03; // Starting guess
  private double stepKp = 5;
  private double stepKv = 0.005;

  private double currentKp, currentKv;
  private double baseCost = Double.MAX_VALUE;
  private double currentCost = 0.0;

  private int exploreDirection = 1; // 1 for positive step, -1 for negative step

  // Profile configuration
  private static final double PROFILE_DURATION_SEC = 2.0;
  private static final double SINE_AMPLITUDE_DEG = 45.0;
  private static final double SINE_FREQUENCY_HZ = 0.5;

  public AutoTuneTurret(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    currentKp = baseKp;
    currentKv = baseKv;
    profileTimer.restart();
    currentCost = 0.0;
    currentState = SearchState.EXPLORE_KP;
  }

  @Override
  public void execute() {
    if (!Turret.turretCalibrationDone) return;

    // 1. Generate standard synthetic profile (Sine Wave)
    double t = profileTimer.get();

    // Position: A * sin(2 * pi * f * t)
    double targetAngle = SINE_AMPLITUDE_DEG * Math.sin(2 * Math.PI * SINE_FREQUENCY_HZ * t);
    Logger.recordOutput("AutoTune/Target", targetAngle);
    // Velocity (Derivative of position): A * 2 * pi * f * cos(2 * pi * f * t)
    double targetVelocity =
        SINE_AMPLITUDE_DEG
            * 2
            * Math.PI
            * SINE_FREQUENCY_HZ
            * Math.cos(2 * Math.PI * SINE_FREQUENCY_HZ * t);

    // 2. Apply current exploratory gains
    turret.setPID(currentKp, 0.0, 0.0);
    double appliedFeedforward = currentKv * targetVelocity;

    turret.setTurretAngleWithFeedforward(targetAngle, appliedFeedforward);

    // 3. Accumulate Cost (Integral Square Error)
    double error = targetAngle - turret.getTurretAngle();
    currentCost += (error * error); // Adds e^2 every 20ms loop

    Logger.recordOutput("AutoTune/CurrentKp", currentKp);
    Logger.recordOutput("AutoTune/CurrentKv", currentKv);
    Logger.recordOutput("AutoTune/AccumulatingCost", currentCost);

    // 4. End of profile run -> Evaluate and step the search algorithm
    if (t >= PROFILE_DURATION_SEC) {
      evaluateAndStepSearch();
      profileTimer.restart();
      currentCost = 0.0; // Reset for the next run
    }
  }

  private void evaluateAndStepSearch() {
    Logger.recordOutput("AutoTune/LastRunCost", currentCost);

    // If this is the very first run, record it as the base cost
    if (baseCost == Double.MAX_VALUE) {
      baseCost = currentCost;
    }

    switch (currentState) {
      case EXPLORE_KP:
        if (currentCost < baseCost) {
          // Improvement found! Keep the new base and move to Kv
          baseCost = currentCost;
          baseKp = currentKp;
          exploreDirection = 1;
          currentState = SearchState.EXPLORE_KV;
          currentKv = MathUtil.clamp(baseKv + (stepKv * exploreDirection), MIN_KV, MAX_KV);
        } else {
          // No improvement. Try the other direction, or move on
          if (exploreDirection == 1) {
            exploreDirection = -1;
            currentKp = MathUtil.clamp(baseKp + (stepKp * exploreDirection), MIN_KP, MAX_KP);
          } else {
            // Both directions failed. Reset to base and move to Kv
            currentKp = baseKp;
            exploreDirection = 1;
            currentState = SearchState.EXPLORE_KV;
            currentKv = MathUtil.clamp(baseKv + (stepKv * exploreDirection), MIN_KV, MAX_KV);
          }
        }
        break;

      case EXPLORE_KV:
        if (currentCost < baseCost) {
          baseCost = currentCost;
          baseKv = currentKv;
          currentState = SearchState.PATTERN_MOVE;
        } else {
          if (exploreDirection == 1) {
            exploreDirection = -1;
            currentKv = MathUtil.clamp(baseKv + (stepKv * exploreDirection), MIN_KV, MAX_KV);
          } else {
            currentKv = baseKv;
            currentState = SearchState.PATTERN_MOVE;
          }
        }
        break;

      case PATTERN_MOVE:
        // A full Hooke & Jeeves pattern move accelerates the search down the gradient.
        // For physical safety, we will just shrink the step sizes and restart the exploratory loop
        // to slowly zero in on the absolute minimum.
        stepKp *= 0.5;
        stepKv *= 0.5;

        currentState = SearchState.EXPLORE_KP;
        exploreDirection = 1;
        currentKp = MathUtil.clamp(baseKp + (stepKp * exploreDirection), MIN_KP, MAX_KP);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    // Print the final optimal gains to the console so you can copy them to TurretConstants!
    System.out.println("Tuning Complete! Optimal kP: " + baseKp + " | Optimal kV: " + baseKv);
  }
}
