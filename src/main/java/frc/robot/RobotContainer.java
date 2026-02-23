package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TrackTarget;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.SetIntakePivotAngle;
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
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Robot container with subsystems, commands, and button mappings. */
public class RobotContainer {
  private final Drive drive;
  private final IntakePivot intakePivot;
  private final Intake intake;
  private final Shooter shooter;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Turret turret;
  private final Hood hood;
  private final Vision vision;
  // private final Vision vision;
  private final CommandXboxController controller = new CommandXboxController(0);
  private Command autoCommand;

  public static Alliance currentAlliance = Alliance.Red;
  public final LoggedDashboardChooser<Alliance> m_allianceChooser;

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
        turret = new Turret(new TurretIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());

        VisionIO staticCameraIO =
            new VisionIOLimelight(
                "limelight",
                () -> RobotState.getInstance().getEstimatedPose().getRotation(),
                () -> RobotState.getInstance().getDriveAngularVelocity());

        VisionIO turretCameraIO =
            new VisionIOLimelight(
                "limelight-turret",
                () -> RobotState.getInstance().getEstimatedPose().getRotation(),
                () -> RobotState.getInstance().getDriveAngularVelocity(),
                () -> {
                  Transform3d turretRotation =
                      new Transform3d(
                          new Translation3d(),
                          new Rotation3d(
                              0.0, 0.0, Units.degreesToRadians(turret.getTurretAngle())));

                  Pose3d turretCameraPose =
                      new Pose3d()
                          .transformBy(VisionConstants.ROBOT_TO_TURRET)
                          .transformBy(turretRotation)
                          .transformBy(VisionConstants.TURRET_TO_CAMERA);

                  Logger.recordOutput("Vision/TurretLLPose", turretCameraPose);
                  return turretCameraPose;
                });
        vision = new Vision(staticCameraIO, turretCameraIO);
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
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        vision = new Vision(new VisionIO() {});
        /*
        new Vision(
            new VisionIOPhotonVisionSim(
                "limelight4",
                VisionConstants.robotToCamera0,
                () -> RobotState.getInstance().getEstimatedPose()));*/

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
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        vision = new Vision(new VisionIO() {});

        break;
    }

    autoCommand = DriveCommands.wheelRadiusCharacterization(drive);
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

    m_allianceChooser = new LoggedDashboardChooser<>("Alliance Chooser");

    m_allianceChooser.addDefaultOption("Red Alliance", Alliance.Red);
    m_allianceChooser.addOption("Blue Alliance", Alliance.Blue);
  }

  public static Alliance getAlliance() {
    return currentAlliance;
  }

  /** Define button-to-command mappings. */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        new DefaultDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX() * 0.75));

    controller
        .rightBumper()
        .toggleOnTrue(
            intake
                .runIntakeCommand(() -> 6.5)
                .alongWith(new SetIntakePivotAngle(intakePivot, 0, true)));

    controller.leftBumper().toggleOnTrue(intake.runIntakeCommand(() -> -6.5));

    Trigger inLaunchingTolerance =
        new Trigger(
            () -> hood.atGoal() && shooter.atGoal() && turret.atGoal());

    controller
        .rightTrigger()
        .whileTrue(
            shooter.shooterRPMTuningCommand(
                () -> LaunchCalculator.getInstance().getParameters().flywheelSpeed()))
        .whileTrue(
            hood.hoodPositionTuningCommand(
                () -> LaunchCalculator.getInstance().getParameters().hoodAngle()))
        .whileTrue(
            new TrackTarget(
                turret,
                () -> LaunchCalculator.getInstance().getParameters().turretAngle().getDegrees(),
                () -> LaunchCalculator.getInstance().getParameters().turretVelocity()))
                .and(() -> LaunchCalculator.getInstance().getParameters().isValid())
                .and(() -> !Turret.wrappingAngle)
                .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling))
        .whileTrue(
            Commands.parallel(
                spindexer.setSpindexerVoltageCommand(() -> 5),
                feeder.setFeederVoltageCommand(() -> 11)));

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
    return autoCommand; // autoChooser.get();
  }
}
