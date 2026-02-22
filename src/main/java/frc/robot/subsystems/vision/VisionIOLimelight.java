// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleSupplier angularVelocitySupplier;
  private final Supplier<Pose3d> cameraPoseSupplier; // Can be null for static cameras

  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleArrayPublisher cameraPosePublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight for a STATIC camera. Relies on the Limelight Web UI for the
   * Camera Pose configuration.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   * @param angularVelocitySupplier Supplier for the current angular velocity of the robot, used for
   *     MegaTag 2.
   */
  public VisionIOLimelight(
      String name, Supplier<Rotation2d> rotationSupplier, DoubleSupplier angularVelocitySupplier) {
    // Call the main constructor, passing null for the camera pose supplier
    this(name, rotationSupplier, angularVelocitySupplier, null);
  }

  /**
   * Creates a new VisionIOLimelight for a DYNAMIC (moving) camera. Overrides the Web UI Camera Pose
   * dynamically over NetworkTables.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   * @param angularVelocitySupplier Supplier for the current angular velocity of the robot, used for
   *     MegaTag 2.
   * @param cameraPoseSupplier Supplier for the dynamic camera pose relative to the robot center.
   */
  public VisionIOLimelight(
      String name,
      Supplier<Rotation2d> rotationSupplier,
      DoubleSupplier angularVelocitySupplier,
      Supplier<Pose3d> cameraPoseSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;
    this.cameraPoseSupplier = cameraPoseSupplier;

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {
          rotationSupplier.get().getDegrees(),
          angularVelocitySupplier.getAsDouble(),
          0.0,
          0.0,
          0.0,
          0.0
        });

    // --- ONLY PUBLISH POSE IF IT IS A DYNAMIC CAMERA ---
    if (cameraPoseSupplier != null) {
      Pose3d camPose = cameraPoseSupplier.get();
      cameraPosePublisher.accept(
          new double[] {
            camPose.getX(),
            -camPose.getY(),
            camPose.getZ(),
            -Units.radiansToDegrees(camPose.getRotation().getX()), // Roll
            -Units.radiansToDegrees(camPose.getRotation().getY()), // Pitch
            Units.radiansToDegrees(camPose.getRotation().getZ()) // Yaw
          });
    }

    NetworkTableInstance.getDefault().flush();

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // (MegaTag 1 & 2 parsing logic remains exactly the same as your original snippet)
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_1));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_2));
    }

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
