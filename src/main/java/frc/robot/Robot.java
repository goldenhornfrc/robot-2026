// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public static boolean isAuto = false;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    switch (Constants.currentMode) {
      case REAL -> {
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }
    SignalLogger.stop();
    AutoLogOutputManager.addObject(RobotState.getInstance());

    Logger.start();
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotContainer.currentAlliance = robotContainer.m_allianceChooser.get();
    SmartDashboard.putString("Selected Alliance", RobotContainer.getAlliance().toString());
    var launchCalculator = LaunchCalculator.getInstance();
    Logger.recordOutput("LaunchCalculator/Parameters", launchCalculator.getParameters());
    Logger.recordOutput(
        "LaunchCalculator/HoodAngleOffsetDeg", launchCalculator.getHoodAngleOffsetDeg());
    String formattedOffset = String.format("%.1f", launchCalculator.getHoodAngleOffsetDeg());
    if (formattedOffset.equals("-0.0")) {
      formattedOffset = "0.0";
    }
    SmartDashboard.putString("Hood Angle Offset", formattedOffset);

    // Clear launching parameters
    launchCalculator.clearLaunchingParameters();
    // Zones.logAllZones();
  }

  @Override
  public void disabledInit() {
    isAuto = false;
    //CommandScheduler.getInstance()
     //   .schedule(robotContainer.ledSubsystem.fallingBlocksCommand(Color.kOrange, 0.1));
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.hood.resetHoodAngle(HoodConstants.kHoodStartingPos);
    robotContainer.intakePivot.resetPivotAngle(IntakeConstants.intakePivotStartingPos);
    isAuto = true;
    CommandScheduler.getInstance().cancelAll();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    isAuto = false;
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.hood.setHoodAngle(0);
    CommandScheduler.getInstance().cancelAll();

    /*
    RobotState.getInstance()
        .resetPose(
            new Pose2d(
                FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(), Rotation2d.fromDegrees(0)));
                */
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    RobotState.getInstance()
        .resetPose(
            new Pose2d(
                FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(),
                Rotation2d.fromDegrees(180)));
    Turret.turretCalibrationDone = true;
  }

  @Override
  public void simulationPeriodic() {}
}
