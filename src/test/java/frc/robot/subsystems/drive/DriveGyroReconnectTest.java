package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DriveGyroReconnectTest {

  private static class SimGyroIO implements GyroIO {
    boolean connected = true;
    Rotation2d yaw = Rotation2d.kZero;
    double currentTime = 0.0;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = connected;
      inputs.yawPosition = yaw;
      inputs.odometryYawPositions = new Rotation2d[] {yaw};
      inputs.odometryYawTimestamps = new double[] {currentTime};
      inputs.yawVelocityRadPerSec = Math.PI / 2.0;
    }
  }

  private static class SimModuleIO implements ModuleIO {
    double drivePosRad = 0.0;
    Rotation2d turnPos = Rotation2d.kZero;
    double speedRadPerSec = 0.0;
    double currentTime = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      drivePosRad += speedRadPerSec * 0.02;
      inputs.odometryTimestamps = new double[] {currentTime};
      inputs.odometryDrivePositionsRad = new double[] {drivePosRad};
      inputs.odometryTurnPositions = new Rotation2d[] {turnPos};
    }
  }

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0); // Initialize WPILib HAL
    RobotState.getInstance().resetPose(new Pose2d());
  }

  @Test
  public void testGyroReconnectMaintainsContinuousRotation() {
    SimGyroIO gyroIO = new SimGyroIO();
    SimModuleIO fl = new SimModuleIO();
    SimModuleIO fr = new SimModuleIO();
    SimModuleIO bl = new SimModuleIO();
    SimModuleIO br = new SimModuleIO();

    // Constant rotation of 90 degrees/sec
    double omegaRadPerSec = Math.PI / 2.0;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, omegaRadPerSec);
    SwerveDriveKinematics kin = new SwerveDriveKinematics(Drive.getModuleTranslations());
    SwerveModuleState[] states = kin.toSwerveModuleStates(chassisSpeeds);

    SimModuleIO[] mods = new SimModuleIO[] {fl, fr, bl, br};
    double[] radiuses =
        new double[] {
          TunerConstants.FrontLeft.WheelRadius,
          TunerConstants.FrontRight.WheelRadius,
          TunerConstants.BackLeft.WheelRadius,
          TunerConstants.BackRight.WheelRadius
        };

    for (int i = 0; i < 4; i++) {
      mods[i].turnPos = states[i].angle;
      mods[i].speedRadPerSec = states[i].speedMetersPerSecond / radiuses[i];
    }

    Drive drive = new Drive(gyroIO, fl, fr, bl, br);

    double t = 0;
    Rotation2d realHeading = Rotation2d.kZero;

    // 1. 2 seconds connected (90 deg/sec * 2 sec = 180 degrees)
    for (int i = 0; i < 100; i++) {
      t += 0.02;
      realHeading = realHeading.plus(Rotation2d.fromRadians(omegaRadPerSec * 0.02));
      gyroIO.currentTime = t;
      gyroIO.yaw = realHeading;
      for (SimModuleIO m : mods) m.currentTime = t;
      drive.periodic();
    }

    // We expect heading to have rotated 180 degrees.
    assertEquals(180.0, RobotState.getInstance().getRotation().getDegrees(), 1.0);

    // 2. 1 second disconnected (relies on wheel deltas: +90 degrees = 270 degrees)
    gyroIO.connected = false;
    for (int i = 0; i < 50; i++) {
      t += 0.02;
      realHeading = realHeading.plus(Rotation2d.fromRadians(omegaRadPerSec * 0.02));
      gyroIO.currentTime = t;
      for (SimModuleIO m : mods) m.currentTime = t;
      drive.periodic();
    }

    // It should have safely integrated the twist using wheel distances alone.
    // -90 degrees is synonymous with 270
    assertEquals(-90.0, RobotState.getInstance().getRotation().getDegrees(), 1.0);

    // 3. 2 seconds re-connected (e.g. gyro reset to 0 right at boot and starts measuring +90 deg/s
    // again)
    gyroIO.connected = true;
    Rotation2d rebootedGyroYaw = Rotation2d.kZero;

    for (int i = 0; i < 100; i++) {
      t += 0.02;
      realHeading = realHeading.plus(Rotation2d.fromRadians(omegaRadPerSec * 0.02));
      rebootedGyroYaw = rebootedGyroYaw.plus(Rotation2d.fromRadians(omegaRadPerSec * 0.02));

      gyroIO.currentTime = t;
      gyroIO.yaw = rebootedGyroYaw;
      for (SimModuleIO m : mods) m.currentTime = t;
      drive.periodic();
    }

    // Total theoretical turn = 180 (connected) + 90 (simulated) + 180 (reconnected) = 450 degrees.
    // 450 mod 360 = 90 degrees!
    assertEquals(90.0, RobotState.getInstance().getRotation().getDegrees(), 1.0);
  }
}
