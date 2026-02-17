// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  private static final Rotation3d cameraRotation =
      new Rotation3d()
          .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(0)))
          .rotateBy(new Rotation3d(0, Units.degreesToRadians(-15), 0));

  public static Transform3d robotToCamera0 =
      new Transform3d(-0.251744, 0.2496, 0.541224, cameraRotation);
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static SimCameraProperties LL4_640_480() {
    var prop = new SimCameraProperties();

    // Approximate focal length from HFOV = 82°
    double hfovRad = Units.degreesToRadians(82.0);
    double fx = (640.0 / 2.0) / Math.tan(hfovRad / 2.0);
    double fy = fx; // assume square pixels
    double cx = 640.0 / 2.0;
    double cy = 480.0 / 2.0;

    // Build intrinsic matrix
    prop.setCalibration(
        640,
        480,
        MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
        VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0));

    // Optional performance settings
    prop.setCalibError(0.0, 0.00);
    prop.setFPS(30);
    prop.setAvgLatencyMs(20);
    prop.setLatencyStdDevMs(8);

    return prop;
  }
}
