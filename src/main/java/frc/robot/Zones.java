package frc.robot;

import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Bounds;
import org.littletonrobotics.junction.Logger;

public class Zones {

  // 20cm padding for safe entry/exit triggers
  private static final double MARGIN = -0.2;

  // MARK: - Bounds Definitions (Strictly Blue Alliance)

  public static final Bounds BLUE_LEFT_TRENCH =
      new Bounds(
          FieldConstants.LeftTrench.openingTopLeft.getX()
              - FieldConstants.LeftTrench.depth
              - MARGIN,
          FieldConstants.LeftTrench.openingTopLeft.getX()
              + FieldConstants.LeftTrench.depth
              + MARGIN,
          FieldConstants.LeftTrench.openingTopRight.getY(), // Smaller Y
          FieldConstants.LeftTrench.openingTopLeft.getY() // Larger Y
          );

  public static final Bounds BLUE_RIGHT_TRENCH =
      new Bounds(
          FieldConstants.RightTrench.openingTopLeft.getX()
              - FieldConstants.RightTrench.depth
              - MARGIN,
          FieldConstants.RightTrench.openingTopLeft.getX()
              + FieldConstants.RightTrench.depth
              + MARGIN,
          FieldConstants.RightTrench.openingTopRight.getY(), // Smaller Y
          FieldConstants.RightTrench.openingTopLeft.getY() // Larger Y
          );

  public static final Bounds[] TRENCH_ZONES = {BLUE_LEFT_TRENCH, BLUE_RIGHT_TRENCH};
  // MARK: - Logging

  /** Logs the rectangular boundaries to AdvantageScope. */
  public static void logAllZones() {
    Logger.recordOutput(
        "Zones/Trenches/Left", AllianceFlipUtil.apply(BLUE_LEFT_TRENCH).getCorners());
    Logger.recordOutput(
        "Zones/Trenches/Right", AllianceFlipUtil.apply(BLUE_RIGHT_TRENCH).getCorners());
  }
}
