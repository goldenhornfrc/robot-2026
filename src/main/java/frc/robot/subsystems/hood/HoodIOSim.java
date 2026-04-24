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
  private static final double MECHANISM_INERTIA = 0.005054;

  private final DCMotorSim hoodSim;

  // Phoenix 6 expects PID error in Rotations
  private final PIDController positionController = new PIDController(0.4, 0.0, 0.0);
  private boolean closedLoop = false;

  // Setpoint mapped to Rotations for TalonFX parity
  private double positionSetpointRotations = 0.0;
  private double appliedVolts = 0.0;

  // Encoder reset offset
  private double angleOffsetRad = 0.0;

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

    // 3. Calculate control loops using Rotations
    if (closedLoop) {
      // Calculating feedforward with actual velocity instead of target velocity causes positive
      // feedback.
      // Since this sim simple-position control doesn't track a profile, we ignore kV/kA to prevent
      // violent oscillation.
      appliedVolts = positionController.calculate(currentRot, positionSetpointRotations);
    }

    // Allow full 12V range for typical FRC motors
    hoodSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    hoodSim.update(0.02);

    inputs.motorConnected = true;
    inputs.positionDegrees = Math.toDegrees(currentRad);
    inputs.velocityDegreesPerSecond = Math.toDegrees(simVelRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
    inputs.tempCelsius = 25.0;
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
}
