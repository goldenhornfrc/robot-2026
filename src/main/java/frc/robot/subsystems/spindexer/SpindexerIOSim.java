// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.spindexer;

/** Very small sim implementation for SpindexerIO. Supports open-loop voltage control. */
public class SpindexerIOSim implements SpindexerIO {
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 0.4;
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void runVolts(double voltage) {
    appliedVolts = voltage;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
