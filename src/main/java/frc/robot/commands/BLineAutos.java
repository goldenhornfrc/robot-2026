package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.SetIntakePivotAngle;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.LaunchCalculator.DesiredAction;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class BLineAutos {

  private final FollowPath.Builder pathBuilder;
  /* Auto state info */
  private boolean shouldShoot = false;

  public BLineAutos(
      FollowPath.Builder pathBuilder,
      Turret turret,
      Hood hood,
      Shooter shooter,
      Intake intake,
      IntakePivot intakePivot,
      Feeder feeder,
      Spindexer spindexer) {
    this.pathBuilder = pathBuilder;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.intake = intake;
    this.intakePivot = intakePivot;
    this.feeder = feeder;
    this.spindexer = spindexer;

    FollowPath.setDoubleLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    FollowPath.setBooleanLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    FollowPath.setPoseLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    FollowPath.setTranslationListLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });
  }

  public Command testAuto() {
    resetAutoStateVariables();
    FollowPath followCommand = (FollowPath) pathBuilder.build(new Path("deneme"));
    FollowPath.registerEventTrigger("start_shoot", () -> shouldShoot = true);
    FollowPath.registerEventTrigger("stop_shoot", () -> shouldShoot = false);

    return Commands.sequence(
        Commands.deadline(
            followCommand,
            Commands.waitUntil(() -> shouldShoot)
                .deadlineFor(new WaitCommand(0.5).andThen(trackHub().alongWith(intakeCommand())))
                .andThen(
                    shootCommandGroup().withDeadline(Commands.waitUntil(() -> !shouldShoot)))));
  }

  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Intake intake;
  private final IntakePivot intakePivot;
  private final Feeder feeder;
  private final Spindexer spindexer;

  private Command trackTarget() {
    return new TrackTarget(
        turret,
        () -> LaunchCalculator.getInstance().getParameters().turretAngle().getDegrees(),
        () -> LaunchCalculator.getInstance().getParameters().turretVelocity());
  }

  private Command trackHub() {
    return new TrackTarget(
        turret,
        () -> {
          var target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d();
          var robotPos = RobotState.getInstance().getEstimatedPose();
          Rotation2d angle =
              target.minus(robotPos.getTranslation()).getAngle().minus(robotPos.getRotation());
          return angle.getDegrees();
        },
        () -> 0);
  }

  private Command homeHood() {
    return new InstantCommand(() -> hood.setHoodAngle(0));
  }

  private Command setShooterRPMDistance() {
    return shooter.shooterRPMTuningCommand(
        () -> LaunchCalculator.getInstance().getParameters().flywheelSpeed());
  }

  private Command setHoodAngleDistance() {
    return hood.hoodPositionTuningCommand(
        () -> LaunchCalculator.getInstance().getParameters().hoodAngle());
  }

  private Command intakeCommand() {
    return intake
        .runIntakeCommand(() -> 7.7)
        .alongWith(new SetIntakePivotAngle(intakePivot, 2.0, true));
  }

  private Command deployIntake() {
    return new InstantCommand(() -> intakePivot.setPivotAngle(10.0));
  }

  private Command intakeWiggle() {
    return new RepeatCommand(
            new SetIntakePivotAngle(intakePivot, 65, true)
                .withTimeout(0.4)
                .andThen(new WaitCommand(0.1))
                .andThen(
                    new SetIntakePivotAngle(intakePivot, 40, true)
                        .withTimeout(0.3)
                        .andThen(new WaitCommand(0.1))))
        .alongWith(IntakeCommands.setIntakeVoltage(6.5, intake));
  }

  private Command enableVision() {
    return new InstantCommand(() -> Vision.allowVisionMeasurements = true);
  }

  private Command disableVision() {
    return new InstantCommand(() -> Vision.allowVisionMeasurements = false);
  }

  private Command setFeedMode() {
    return new InstantCommand(
        () -> LaunchCalculator.getInstance().desiredAction = DesiredAction.FEED);
  }

  private Command setShootMode() {
    return new InstantCommand(
        () -> LaunchCalculator.getInstance().desiredAction = DesiredAction.SHOOT);
  }

  private Command feedBalls() {
    return Commands.runEnd(
        () -> {
          Debouncer atGoalDebouncer = new Debouncer(0.25, DebounceType.kFalling);

          if (LaunchCalculator.getInstance().getParameters().isValid()
              && atGoalDebouncer.calculate(hood.atGoal() && shooter.atGoal() && turret.atGoal())
              && !Turret.wrappingAngle) {
            feeder.runVelocity(1000);
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
        spindexer);
  }

  private Command shootCommandGroup() {
    return Commands.parallel(
        trackTarget(),
        setShooterRPMDistance(),
        setHoodAngleDistance(),
        //setShootMode(),
        new WaitCommand(0.15).andThen(feedBalls()));
  }

  private Command waitUntilTranslationalSegment(FollowPath followCommand, int segment) {
    return Commands.waitUntil(() -> followCommand.getCurrentTranslationElementIndex() >= segment);
  }

  private Command waitUntilRotationalSegment(FollowPath followCommand, int segment) {
    return Commands.waitUntil(() -> followCommand.getCurrentRotationElementIndex() >= segment);
  }

  private void resetAutoStateVariables() {
    shouldShoot = false;
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
        double hoodAngleDeg = 80.0 - hood.getHoodAngle();
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
}
