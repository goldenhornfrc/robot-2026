// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// Use of this source code is governed by a BSD license
// that can be found in the LICENSE file at the root directory.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.feeder.FeederCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.SetIntakePivotAngle;
import frc.robot.commands.spindexer.SpindexerCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;

/** Robot container with subsystems, commands, and button mappings. */
public class RobotContainer {
  private final Drive drive;
  private final IntakePivot intakePivot;
  private final Intake intake;
  private final Shooter shooter;
  private final Spindexer spindexer;
  private final Feeder feeder;
  // private final Vision vision;
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        spindexer = new Spindexer(new SpindexerIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});

        feeder = new Feeder(new FeederIO() {});

        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});

        feeder = new Feeder(new FeederIO() {});

        break;
    }

    /*
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
    configureButtonBindings();
  }

  /** Define button-to-command mappings. */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .y()
        .whileTrue(
            shooter
                .shooterVoltageCommand(() -> 6.0)
                .alongWith(SpindexerCommands.setSpindexerVoltage(6.0, spindexer))
                .alongWith(FeederCommands.setFeederVoltage(10, feeder)));

    controller.leftBumper().whileTrue(IntakeCommands.setIntakePivotVoltage(1.0, intakePivot));
    controller.rightBumper().whileTrue(IntakeCommands.setIntakePivotVoltage(-1.0, intakePivot));

    controller.rightTrigger().whileTrue(IntakeCommands.setIntakeVoltage(5.0, intake));

    controller
        .a()
        .whileTrue(
            new RepeatCommand(
                    new SetIntakePivotAngle(intakePivot, 45, true)
                        .withTimeout(0.3)
                        .andThen(new WaitCommand(0.1))
                        .andThen(
                            new SetIntakePivotAngle(intakePivot, 20, true)
                                .withTimeout(0.3)
                                .andThen(new WaitCommand(0.1))))
                .alongWith(IntakeCommands.setIntakeVoltage(5.0, intake)));
  }

  public Command getAutonomousCommand() {
    return null; // autoChooser.get();
  }
}
