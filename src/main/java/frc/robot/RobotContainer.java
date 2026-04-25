package frc.robot;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation; // <--- ADDED IMPORT
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState.VisionObservation;
import frc.robot.commands.BLineAutos;
import frc.robot.commands.TrackTarget;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.SetIntakeDeployPos;
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
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeDeployIO;
import frc.robot.subsystems.intake.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
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
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Robot container with subsystems, commands, and button mappings. */
public class RobotContainer {
  private final Drive drive;
  public final IntakeDeploy intakeDeploy;
  public final Intake intake;
  private final Shooter shooter;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Turret turret;
  public final Hood hood;
  private final Vision vision;
  public final LEDSubsystem ledSubsystem;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  private static boolean allowAutoAlign = true;
  private static boolean allowAutoSwitchTarget = true;

  public static Alliance currentAlliance = Alliance.Red;
  public final LoggedDashboardChooser<Alliance> m_allianceChooser;

  private final LoggedDashboardChooser<Command> autoChooser;

  private BLineAutos bLineAutos;

  private static final LoggedTunableNumber feederTuningRpm =
      new LoggedTunableNumber("Feeder/TuningRPM", 1200.0);
  private static final LoggedTunableNumber spindexerTuningRpm =
      new LoggedTunableNumber("Spindexer/TuningRPM", 1200.0);

  public RobotContainer() {
    ledSubsystem = new LEDSubsystem();

    // --- ADDED: LED Default Command ---
    // Make Falling Blocks the default so it runs whenever nothing else is requiring the LEDs
    // (e.g., Disabled & Calibrated). ignoringDisable(true) ensures it can run while disabled.

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
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
                () -> RobotState.getInstance().getDriveAngularVelocity(),
                () -> {
                  final Rotation3d cameraRot =
                      new Rotation3d(0, Units.degreesToRadians(-15.0), Units.degreesToRadians(3.2));
                  final Pose3d cameraPose = new Pose3d(-0.269352, 0.1087, 0.510, cameraRot);
                  return cameraPose;
                });

        VisionIO backCameraIO =
            new VisionIOLimelight(
                "limelight-back",
                () -> RobotState.getInstance().getEstimatedPose().getRotation(),
                () -> RobotState.getInstance().getDriveAngularVelocity(),
                () -> {
                  final Rotation3d cameraRot =
                      new Rotation3d(0, Units.degreesToRadians(-15.0), Math.PI);
                  final Pose3d cameraPose = new Pose3d(-0.31478, 0.2511, 0.508, cameraRot);
                  return cameraPose;
                });

        VisionIO sideCameraIO =
            new VisionIOLimelight(
                "limelight-side",
                () -> RobotState.getInstance().getEstimatedPose().getRotation(),
                () -> RobotState.getInstance().getDriveAngularVelocity(),
                () -> {
                  final Rotation3d cameraRot = new Rotation3d(0, Units.degreesToRadians(15.0), 0.0);
                  final Pose3d cameraPose = new Pose3d(0.0262, -0.037019, 0.373, cameraRot);
                  return cameraPose;
                });

        vision = new Vision(staticCameraIO, backCameraIO); // TODO: add side camera
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        spindexer = new Spindexer(new SpindexerIOSim());

        feeder = new Feeder(new FeederIOSim());
        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
        vision = new Vision(new VisionIO() {});
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});

        feeder = new Feeder(new FeederIO() {});
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        vision = new Vision(new VisionIO() {});

        break;
    }

    bLineAutos =
        new BLineAutos(
            drive.pathBuilder, turret, hood, shooter, intake, intakeDeploy, feeder, spindexer);

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
            .runIntakeCommand(() -> 10)
            .alongWith(
                new SetIntakeDeployPos(
                    intakeDeploy, IntakeConstants.intakeDeployExtendLimitPos, true)));

    NamedCommands.registerCommand(
        "DeployIntake",
        new InstantCommand(
            () ->
                intakeDeploy.setPivotPos(
                    IntakeConstants
                        .intakeDeployExtendLimitPos))); // TODO: add intake deploy command

    NamedCommands.registerCommand(
        "IntakeWiggle",
        new RepeatCommand(
                new SetIntakeDeployPos(intakeDeploy, 2.8, true)
                    .withTimeout(1.0)
                    .andThen(new WaitCommand(0.1))
                    .andThen(
                        new SetIntakeDeployPos(intakeDeploy, 1.3, true)
                            .withTimeout(1.0)
                            .andThen(new WaitCommand(0.1))))
            .alongWith(IntakeCommands.setIntakeVoltage(10, intake)));

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
                feeder.runVelocity(2000);
                spindexer.setVoltage(5);
                simBallShoot();
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

    // autoChooser.addOption("BLine Test Auto", bLineAutos.testAuto());
    autoChooser.addDefaultOption(
        "Reset Sensors",
        new InstantCommand(() -> RobotState.getInstance().resetPose(new Pose2d())));

    autoChooser.addOption(
        "Shooter SysId",
        shooter
            .sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward)
            .andThen(
                new WaitCommand(1.5)
                    .andThen(
                        shooter
                            .sysIdQuasistatic(Direction.kReverse)
                            .andThen(
                                new WaitCommand(1.5)
                                    .andThen(
                                        shooter
                                            .sysIdDynamic(Direction.kForward)
                                            .andThen(
                                                new WaitCommand(1.5)
                                                    .andThen(
                                                        shooter
                                                            .sysIdDynamic(Direction.kReverse)
                                                            .andThen(
                                                                new InstantCommand(
                                                                    () ->
                                                                        SignalLogger
                                                                            .stop())))))))));

    autoChooser.addOption(
        "Shooter Quasistatic SysId",
        shooter.sysIdQuasistatic(
            edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
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
                .runIntakeCommand(() -> 10)
                .alongWith(
                    new SetIntakeDeployPos(
                        intakeDeploy, IntakeConstants.intakeDeployExtendLimitPos, true)));

    controller.leftBumper().toggleOnTrue(intake.runIntakeCommand(() -> -8.0));

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
        .whileTrue(
            Commands.run(
                () -> {
                  intake.setVoltage(10);
                  intake.setRunning(true);
                }))
        .and(() -> LaunchCalculator.getInstance().getParameters().isValid())
        .and(() -> !Turret.wrappingAngle)
        .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling))
        .whileTrue(
            Commands.parallel(
                feeder.runFeederVelocityCommand(() -> 1500.0),
                spindexer.setSpindexerVoltageCommand(() -> 5.0),
                simBallCommand()));

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
                    new SetIntakeDeployPos(intakeDeploy, 2.8, true)
                        .withTimeout(1.0)
                        .andThen(new WaitCommand(0.1))
                        .andThen(
                            new SetIntakeDeployPos(intakeDeploy, 1.3, true)
                                .withTimeout(1.0)
                                .andThen(new WaitCommand(0.1))))
                .alongWith(IntakeCommands.setIntakeVoltage(10, intake)));

    testController.y().whileTrue(spindexer.runSpindexerVelocityCommand(spindexerTuningRpm::get));

    operator
        .cross()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (RobotContainer.getAlliance() == Alliance.Red) {
                    RobotState.getInstance()
                        .resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)));
                  } else {
                    RobotState.getInstance().resetPose(new Pose2d());
                  }
                }));

    operator.triangle().onTrue(new SetIntakeDeployPos(intakeDeploy, 1.3, true));
    operator.R1().whileTrue(feeder.setFeederVoltageCommand(() -> -10));

    // 3170 rpm 18.8 deg tower preset

    operator
        .L1()
        .whileTrue(shooter.shooterRPMTuningCommand(() -> 3170.0))
        .whileTrue(hood.hoodPositionTuningCommand(() -> 18.8))
        .whileTrue(new TrackTarget(turret, () -> 0.0, () -> 0.0))
        .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling))
        .whileTrue(
            Commands.parallel(
                spindexer.setSpindexerVoltageCommand(() -> 5),
                feeder.setFeederVoltageCommand(() -> 10.0)));
    // 2800 rpm 2 deg yapisik atma
    operator
        .L2()
        .whileTrue(shooter.shooterRPMTuningCommand(() -> 2800.0))
        .whileTrue(hood.hoodPositionTuningCommand(() -> 4.0))
        .whileTrue(new TrackTarget(turret, () -> 0.0, () -> 0.0))
        .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling))
        .whileTrue(
            Commands.parallel(
                spindexer.setSpindexerVoltageCommand(() -> 5),
                feeder.setFeederVoltageCommand(() -> 10.0)));

    // ==========================================
    //            LED STATE LOGIC
    // ==========================================

    Trigger isAuto = new Trigger(DriverStation::isAutonomousEnabled);
    Trigger isTeleop = new Trigger(DriverStation::isTeleopEnabled);
    Trigger isDisabled = new Trigger(() -> Robot.isDisabled);

    // We reuse your rightTrigger input as the 'isShooting' intent
    Trigger isShooting = controller.rightTrigger();
    Trigger isIntaking = new Trigger(intake::getRunning);

    Trigger isCalibrated = new Trigger(() -> Turret.turretCalibrationDone);
    Trigger isUncalibrated = isCalibrated.negate();

    // 1. Auto: Rainbow Scroll
    isAuto.whileTrue(ledSubsystem.rainbowScrollCommand(150));

    // 2. Teleop Enabled & NOT Shooting: Solid Purple
    isTeleop
        .and(isShooting.negate().and(isIntaking.negate()))
        .whileTrue(ledSubsystem.solidColorCommand(Color.kPurple));

    isTeleop
        .and(isShooting.negate())
        .and(isIntaking)
        .whileTrue(ledSubsystem.strobeCommand(Color.kBlue));
    // 3. Teleop Shooting & NOT at Goal: Solid Red
    isTeleop
        .and(isShooting)
        .and(inLaunchingTolerance.negate())
        .whileTrue(ledSubsystem.solidColorCommand(Color.kRed));

    // 4. Teleop Shooting & AT Goal: Strobe Green
    isTeleop
        .and(isShooting)
        .and(inLaunchingTolerance)
        .whileTrue(ledSubsystem.strobeCommand(Color.kGreen));

    // 5. Disabled & Uncalibrated: Strobe Red
    isDisabled
        .and(isUncalibrated)
        .whileTrue(ledSubsystem.strobeCommand(Color.kRed).ignoringDisable(true));

    isCalibrated.onTrue(
        ledSubsystem
            .strobeCommand(Color.kGreen)
            .withTimeout(1.8)
            .ignoringDisable(true)
            .andThen(ledSubsystem.twoColorScrollCommand(100)));
  }

  private Rotation2d getClosestAlignment(Rotation2d currentHeading) {
    return currentHeading.getCos() >= 0.0 ? Rotation2d.kZero : Rotation2d.fromDegrees(180.0);
  }

  private void simBallShoot() {
    if (Robot.isSimulation() && Robot.ballSim != null && Math.random() < 0.12) {
      var params = LaunchCalculator.getInstance().getParameters();
      if (params.isValid()) {
        Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
        Rotation2d launchHeading =
            robotPose.getRotation().plus(Rotation2d.fromDegrees(turret.getTurretAngle()));

        Translation2d launchTurretPose =
            robotPose
                .transformBy(GeomUtil.toTransform2d(VisionConstants.ROBOT_TO_TURRET))
                .getTranslation();

        Translation3d launcherPosition =
            new Translation3d(
                launchTurretPose.getX(),
                launchTurretPose.getY(),
                0.49); // TODO: Replace with actual launch position

        double rpm = params.flywheelSpeed();
        double speed =
            (rpm / 60.0) * 2 * Math.PI * 0.0508 * 0.43; // TODO: Calibrate conversion to m/s
        double hoodAngleDeg = 70.0 - hood.getHoodAngle();
        var robotVel = RobotState.getInstance().getFieldVelocity();
        double vx =
            launchHeading.getCos() * speed * Math.cos(Math.toRadians(hoodAngleDeg))
                + robotVel.vxMetersPerSecond;
        double vy =
            launchHeading.getSin() * speed * Math.cos(Math.toRadians(hoodAngleDeg))
                + robotVel.vyMetersPerSecond;
        double vz = speed * Math.sin(Math.toRadians(hoodAngleDeg));
        Translation3d launchVelocity = new Translation3d(vx, vy, vz);

        double spinRPM = rpm * 0.5; // TODO: Tune spin transfer

        Robot.ballSim.launchBall(launcherPosition, launchVelocity, spinRPM);
      }
    }
  }

  private Command simBallCommand() {
    return Commands.run(
        () -> {
          simBallShoot();
        });
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
