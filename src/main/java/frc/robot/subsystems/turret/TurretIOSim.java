// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation IO implementation for the turret. Models the turret as a simple DC motor drivetrain
 * with a gearbox and a PID position controller. Designed to mirror the behavior of the
 * TalonFX-based `TurretIOTalonFX` for simulation and unit tests.
 */
public class TurretIOSim implements TurretIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  private static final double MECHANISM_INERTIA = 0.069588;

  private final DCMotorSim turretSim;

  // Controllers - Configured for Phoenix 6 Units (Rotations)
  private final PIDController positionController = new PIDController(121.25, 0.0, 0.0);

  // NEW: WPILib SimpleMotorFeedforward
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0225, 0.0);

  private boolean closedLoop = false;
  private double positionSetpointRotations = 0.0;
  private double appliedVolts = 0.0;
  private double arbitraryFFVolts = 0.0;

  // NEW: Target velocity for internal feedforward (simulating MotionMagic/ProfiledPID)
  private double targetVelocityRps = 0.0;

  private double angleOffsetRad = 0.0;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX, MECHANISM_INERTIA, TurretConstants.TURRET_SENSOR_TO_MECHANISM_RATIO),
            GEARBOX);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double simPosRad = turretSim.getAngularPositionRad();
    double simVelRadPerSec = turretSim.getAngularVelocityRadPerSec();
    double currentRad = simPosRad + angleOffsetRad;

    double currentRot = currentRad / (2.0 * Math.PI);

    if (closedLoop) {
      // 1. Calculate internal feedforward using the TARGET velocity (Fixes the phase lag!)
      double internalFFVolts = 0; // feedforward.calculate(targetVelocityRps);

      // 2. Combine PID, internal FF, and external arbitrary FF
      appliedVolts =
          positionController.calculate(currentRot, positionSetpointRotations)
              + internalFFVolts
              + arbitraryFFVolts;
    }

    turretSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    turretSim.update(0.02);

    inputs.motorConnected = true;
    inputs.positionDegrees = Math.toDegrees(currentRad);
    inputs.velocityDegreesPerSecond = Math.toDegrees(simVelRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());
    inputs.tempCelsius = 25.0;

    // Clear the one-time external feedforward request so it doesn't get stuck if the command ends
    arbitraryFFVolts = 0.0;
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVolts = voltage;
  }

  @Override
  public void setTurretAngle(double angle) {
    closedLoop = true;
    positionSetpointRotations = angle / 360.0;
    targetVelocityRps = 0.0; // No internal velocity target
    positionController.setSetpoint(positionSetpointRotations);
  }

  @Override
  public void setTurretAngle(double angle, double cruiseVel, double acceleration) {
    closedLoop = true;
    positionSetpointRotations = angle / 360.0;
    targetVelocityRps = cruiseVel / 360.0; // Convert deg/s to Phoenix 6 rot/s
    positionController.setSetpoint(positionSetpointRotations);
  }

  @Override
  public void setTurretAngleWithFeedforward(double angle, double ff) {
    setTurretAngle(angle);
    // Overwrite the external voltage request (This comes from your TrackTarget command)
    this.arbitraryFFVolts = ff;
  }

  @Override
  public void resetTurretAngle(double angle) {
    double desiredRad = Math.toRadians(angle);
    double simPos = turretSim.getAngularPositionRad();
    angleOffsetRad = desiredRad - simPos;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    positionController.setP(kP);
    positionController.setI(kI);
    positionController.setD(kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    // Reinstantiate the feedforward object with the new constants
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }
}
