package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.VisionObservation;
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
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.LaunchCalculator.DesiredAction;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Robot container with subsystems, commands, and button mappings. */
public class RobotContainer {
  private final Drive drive;
  public final IntakePivot intakePivot;
  private final Intake intake;
  private final Shooter shooter;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Turret turret;
  public final Hood hood;
  private final Vision vision;
  public final LEDSubsystem ledSubsystem;
  // private final Vision vision;
  private final CommandXboxController controller = new CommandXboxController(0);

  private static boolean allowAutoAlign = true;
  private static boolean allowAutoSwitchTarget = true;

  public static Alliance currentAlliance = Alliance.Red;
  public final LoggedDashboardChooser<Alliance> m_allianceChooser;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    ledSubsystem = new LEDSubsystem();
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
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        spindexer = new Spindexer(new SpindexerIOSim());

        feeder = new Feeder(new FeederIOSim());
        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
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

    NamedCommands.registerCommand(
        "TrackTarget",
        new TrackTarget(
            turret,
            () -> LaunchCalculator.getInstance().getParameters().turretAngle().getDegrees(),
            () -> LaunchCalculator.getInstance().getParameters().turretVelocity()));

    NamedCommands.registerCommand(
        "TrackHub",
        new TrackTarget(
            turret,
            () -> {
              var target =
                  AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d();
              var robotPos = RobotState.getInstance().getEstimatedPose();
              Rotation2d angle =
                  target.minus(robotPos.getTranslation()).getAngle().minus(robotPos.getRotation());
              return angle.getDegrees();
            },
            () -> 0));

    NamedCommands.registerCommand("HomeHood", new InstantCommand(() -> hood.setHoodAngle(0)));

    NamedCommands.registerCommand(
        "SetShooterRPMDistance",
        shooter.shooterRPMTuningCommand(
            () -> LaunchCalculator.getInstance().getParameters().flywheelSpeed()));

    NamedCommands.registerCommand(
        "SetHoodAngleDistance",
        hood.hoodPositionTuningCommand(
            () -> LaunchCalculator.getInstance().getParameters().hoodAngle()));

    NamedCommands.registerCommand(
        "IntakeCommand",
        intake
            .runIntakeCommand(() -> 7.7)
            .alongWith(new SetIntakePivotAngle(intakePivot, 10, true)));

    NamedCommands.registerCommand(
        "DeployIntake", new InstantCommand(() -> intakePivot.setPivotAngle(10.0)));

    NamedCommands.registerCommand(
        "IntakeWiggle",
        new RepeatCommand(
                new SetIntakePivotAngle(intakePivot, 65, true)
                    .withTimeout(0.4)
                    .andThen(new WaitCommand(0.1))
                    .andThen(
                        new SetIntakePivotAngle(intakePivot, 40, true)
                            .withTimeout(0.3)
                            .andThen(new WaitCommand(0.1))))
            .alongWith(IntakeCommands.setIntakeVoltage(6.5, intake)));

    NamedCommands.registerCommand(
        "EnableVision", new InstantCommand(() -> Vision.allowVisionMeasurements = true));

    NamedCommands.registerCommand(
        "DisableVision", new InstantCommand(() -> Vision.allowVisionMeasurements = false));

    NamedCommands.registerCommand(
        "SetFeedMode",
        new InstantCommand(
            () -> LaunchCalculator.getInstance().desiredAction = DesiredAction.FEED));
    NamedCommands.registerCommand(
        "SetShootMode",
        new InstantCommand(
            () -> LaunchCalculator.getInstance().desiredAction = DesiredAction.SHOOT));

    NamedCommands.registerCommand(
        "FeedBalls",
        Commands.runEnd(
            () -> {
              Debouncer atGoalDebouncer = new Debouncer(0.25, DebounceType.kFalling);

              if (LaunchCalculator.getInstance().getParameters().isValid()
                  && atGoalDebouncer.calculate(hood.atGoal() && shooter.atGoal() && turret.atGoal())
                  && !Turret.wrappingAngle) {
                feeder.setVoltage(10);
                spindexer.setVoltage(5);
              } else {
                feeder.stop();
                spindexer.stop();
              }
            },
            () -> {
              feeder.stop();
              spindexer.stop();
            },
            feeder,
            spindexer));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addDefaultOption(
        "Reset Sensors",
        new InstantCommand(() -> RobotState.getInstance().resetPose(new Pose2d())));

    configureButtonBindings();

    m_allianceChooser = new LoggedDashboardChooser<>("Alliance Chooser");

    m_allianceChooser.addDefaultOption("Red Alliance", Alliance.Red);
    m_allianceChooser.addOption("Blue Alliance", Alliance.Blue);

    RobotState.getInstance()
        .addVisionObservation(
            new VisionObservation(
                0, Pose2d.kZero, new Matrix<>(VecBuilder.fill(1, 1, 1)), "Limelight0 MEGATAG_1"));

    RobotState.getInstance()
        .addVisionObservation(
            new VisionObservation(
                0, Pose2d.kZero, new Matrix<>(VecBuilder.fill(1, 1, 1)), "Limelight0 MEGATAG_2"));

    RobotState.getInstance()
        .addVisionObservation(
            new VisionObservation(
                0, Pose2d.kZero, new Matrix<>(VecBuilder.fill(1, 1, 1)), "Limelight1 MEGATAG_1"));

    RobotState.getInstance()
        .addVisionObservation(
            new VisionObservation(
                0, Pose2d.kZero, new Matrix<>(VecBuilder.fill(1, 1, 1)), "Limelight1 MEGATAG_2"));
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
            () -> -controller.getRightX()));

    controller
        .rightBumper()
        .toggleOnTrue(
            intake
                .runIntakeCommand(() -> 6.5)
                .alongWith(new SetIntakePivotAngle(intakePivot, 10, true)));

    controller.leftBumper().toggleOnTrue(intake.runIntakeCommand(() -> -6.5));

    Trigger inLaunchingTolerance =
        new Trigger(() -> hood.atGoal() && shooter.atGoal() && turret.atGoal());

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
                feeder.setFeederVoltageCommand(() -> 10)
                ));


    Trigger inAllianceZoneTrigger =
        new Trigger(
            () -> {
              var alliance = RobotContainer.getAlliance();
              var robotX = RobotState.getInstance().getEstimatedPose().getX();
              final double MARGIN = 0.2;
              return alliance == Alliance.Red
                  ? robotX >= FieldConstants.LinesVertical.oppAllianceZone - MARGIN
                  : robotX <= FieldConstants.LinesVertical.allianceZone + MARGIN;
            });
    Trigger allowAutoSwitchTargetTrigger =
        new Trigger(() -> allowAutoSwitchTarget == true && !Robot.isAuto);

    Trigger allowAutoAlignTrigger = new Trigger(() -> allowAutoAlign == true && !Robot.isAuto);

    /*

    DoubleSupplier lowerTargetY =
        () ->
            RobotContainer.getAlliance() == Alliance.Red
                ? FieldConstants.LeftTrench.centerYPos
                : FieldConstants.RightTrench.centerYPos;

    DoubleSupplier upperTargetY =
        () ->
            RobotContainer.getAlliance() == Alliance.Red
                ? FieldConstants.RightTrench.centerYPos
                : FieldConstants.LeftTrench.centerYPos;

    Trigger inLowerYRange =
        new Trigger(
            () -> {
              double robotY = RobotState.getInstance().getEstimatedPose().getY();
              double center = lowerTargetY.getAsDouble();
              return Math.abs(robotY - center) <= 0.6;
            });

    Trigger inUpperYRange =
        new Trigger(
            () -> {
              double robotY = RobotState.getInstance().getEstimatedPose().getY();
              double center = upperTargetY.getAsDouble();
              return Math.abs(robotY - center) <= 0.6;
            });

    // 3. Create the Path Prediction Triggers
    Trigger passesLowerTrench =
        new Trigger(
            () ->
                RobotState.getInstance()
                    .willPassThroughBounds(AllianceFlipUtil.apply(Zones.BLUE_RIGHT_TRENCH), 0.5));

    Trigger passesUpperTrench =
        new Trigger(
            () ->
                RobotState.getInstance()
                    .willPassThroughBounds(AllianceFlipUtil.apply(Zones.BLUE_LEFT_TRENCH), 0.5));

    // 4. Combine them into your final Triggers!
    Trigger shouldAlignLowerTrenchTrigger = passesLowerTrench.and(inLowerYRange);
    Trigger shouldAlignUpperTrenchTrigger = passesUpperTrench.and(inUpperYRange);

    shouldAlignLowerTrenchTrigger
        .debounce(0.25, DebounceType.kFalling)
        .and(allowAutoAlignTrigger)
        .onTrue(
            new InstantCommand(
                () -> {
                  //Drive.setTargetHeading(
                  //    getClosestAlignment(RobotState.getInstance().getRotation()));
                  Drive.setTargetYPos(lowerTargetY.getAsDouble());
                  Drive.setDriveState(DriveState.TRENCH_ALIGN);
                }))
        .onFalse(new InstantCommand(() -> Drive.setDriveState(DriveState.TELEOP_DRIVE)));

    shouldAlignUpperTrenchTrigger
        .debounce(0.25, DebounceType.kFalling)
        .and(allowAutoAlignTrigger)
        .onTrue(
            new InstantCommand(
                () -> {
                  //Drive.setTargetHeading(
                  //    getClosestAlignment(RobotState.getInstance().getRotation()));
                  Drive.setTargetYPos(upperTargetY.getAsDouble());
                  Drive.setDriveState(DriveState.TRENCH_ALIGN);
                }))
        .onFalse(new InstantCommand(() -> Drive.setDriveState(DriveState.TELEOP_DRIVE)));
    */

    inAllianceZoneTrigger
        .and(allowAutoSwitchTargetTrigger)
        .onTrue(
            new InstantCommand(
                () -> LaunchCalculator.getInstance().desiredAction = DesiredAction.SHOOT))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (allowAutoSwitchTarget && !Robot.isAuto) {
                    LaunchCalculator.getInstance().desiredAction = DesiredAction.FEED;
                  }
                }));

    controller
        .a()
        .whileTrue(
            new RepeatCommand(
                    new SetIntakePivotAngle(intakePivot, 65, true)
                        .withTimeout(0.4)
                        .andThen(new WaitCommand(0.1))
                        .andThen(
                            new SetIntakePivotAngle(intakePivot, 40, true)
                                .withTimeout(0.3)
                                .andThen(new WaitCommand(0.1))))
                .alongWith(IntakeCommands.setIntakeVoltage(6.5, intake)))
        .onFalse(new SetIntakePivotAngle(intakePivot, 10, true));
  }

  private Rotation2d getClosestAlignment(Rotation2d currentHeading) {
    return currentHeading.getCos() >= 0.0 ? Rotation2d.kZero : Rotation2d.fromDegrees(180.0);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
