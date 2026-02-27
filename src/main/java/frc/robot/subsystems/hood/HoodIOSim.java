// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simple simulation IO for the Hood subsystem using DCMotorSim. */
public class HoodIOSim implements HoodIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44(1);

  // Tunable guessed inertia for the hood mechanism
  private static final double MECHANISM_INERTIA = 0.004754;

  private final DCMotorSim hoodSim;

  // Phoenix 6 expects PID error in Rotations
  private final PIDController positionController = new PIDController(400.0, 0.0, 6.0);
  private boolean closedLoop = false;

  // Setpoint mapped to Rotations for TalonFX parity
  private double positionSetpointRotations = 0.0;
  private double appliedVolts = 0.0;

  private double kS = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  // Encoder reset offset
  private double angleOffsetRad = 0.0;
  private double lastVelocityRps = 0.0; // Track RPS instead of Rad/s

  public HoodIOSim() {
    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX, MECHANISM_INERTIA, HoodConstants.HOOD_SENSOR_TO_MECHANISM_RATIO),
            GEARBOX);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // 1. Get WPILib native states (Radians)
    double simPosRad = hoodSim.getAngularPositionRad();
    double simVelRadPerSec = hoodSim.getAngularVelocityRadPerSec();
    double currentRad = simPosRad + angleOffsetRad;

    // 2. Convert to TalonFX units (Rotations)
    double currentRot = currentRad / (2.0 * Math.PI);
    double currentRps = simVelRadPerSec / (2.0 * Math.PI);

    // 3. Calculate acceleration in RPS^2
    double accelRps2 = (currentRps - lastVelocityRps) / 0.02;

    // 4. Calculate control loops using Rotations
    if (closedLoop) {
      double ff = kS * Math.signum(currentRps) + kV * currentRps + kA * accelRps2;
      appliedVolts = positionController.calculate(currentRot, positionSetpointRotations) + ff;
    }

    hoodSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    hoodSim.update(0.02);

    inputs.motorConnected = true;
    inputs.positionDegrees = Math.toDegrees(currentRad);
    inputs.velocityDegreesPerSecond = Math.toDegrees(simVelRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
    inputs.tempCelsius = 25.0;

    lastVelocityRps = currentRps;
  }

  @Override
  public void setHoodAngle(double angle) {
    closedLoop = true;
    // Store setpoint in Rotations for the PID controller
    positionSetpointRotations = angle / 360.0;
    positionController.setSetpoint(positionSetpointRotations);
  }

  @Override
  public void setHoodAngle(double angle, double cruiseVel, double acceleration) {
    // Ignore cruise/accel in this simple sim
    setHoodAngle(angle);
  }

  @Override
  public void resetHoodAngle(double angle) {
    double desiredRad = Math.toRadians(angle);
    double simPos = hoodSim.getAngularPositionRad();
    angleOffsetRad = desiredRad - simPos;
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVolts = voltage;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    positionController.setP(kP);
    positionController.setI(kI);
    positionController.setD(kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }
}
