package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Small compatibility wrapper exposing factory methods to create the default drive command. */
public final class DefaultDriveHeading {
  private DefaultDriveHeading() {}

  public static Command create(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return DriveCommands.defaultDrive(drive, xSupplier, ySupplier, omegaSupplier);
  }

  public static Command create(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return DriveCommands.defaultDrive(drive, xSupplier, ySupplier, omegaSupplier, rotationSupplier);
  }
}
