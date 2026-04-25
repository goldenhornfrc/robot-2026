// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Sim implementation for SpindexerIO supporting velocity control. */
public class SpindexerIOSim implements SpindexerIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);
  private static final double MECHANISM_INERTIA = 0.001; // Guess for spindexer inertia

  private final FlywheelSim spindexerSim;

  private final PIDController velocityController = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;

  private double targetRps = 0.0;
  private double appliedVolts = 0.0;

  private double kS = 0.0;
  private double kA = 0.0;

  private double lastTargetRps = 0.0;
  private double simKv = 0.0;

  public SpindexerIOSim() {
    var plant =
        LinearSystemId.createFlywheelSystem(
            GEARBOX, MECHANISM_INERTIA, SpindexerConstants.SENSOR_TO_MECHANISM_RATIO);
    spindexerSim = new FlywheelSim(plant, GEARBOX, 0.0);
    simKv = -plant.getA(0, 0) / plant.getB(0, 0) * 2.0 * Math.PI;
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    double simVelRadPerSec = spindexerSim.getAngularVelocityRadPerSec();
    double currentRps = simVelRadPerSec / (2.0 * Math.PI);

    if (closedLoop) {
      double targetAccelRps2 = (targetRps - lastTargetRps) / 0.02;
      double ff = kS * Math.signum(targetRps) + simKv * targetRps + kA * targetAccelRps2;
      double pidVolts = velocityController.calculate(currentRps, targetRps);
      appliedVolts = pidVolts + ff;
    }

    spindexerSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    spindexerSim.update(0.02);

    inputs.motorConnected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPM = currentRps * 60.0;
    inputs.supplyCurrentAmps = Math.abs(spindexerSim.getCurrentDrawAmps());
    inputs.tempCelsius = 25.0;

    lastTargetRps = targetRps;
  }

  @Override
  public void runVolts(double voltage) {
    closedLoop = false;
    appliedVolts = voltage;
  }

  @Override
  public void runVelocity(double velocityRPM) {
    closedLoop = true;
    targetRps = velocityRPM / 60.0;
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
    this.kA = kA;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
