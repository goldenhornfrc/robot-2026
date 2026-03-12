// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public static boolean allowVisionMeasurements = true;

  // 1. GC Optimization: Pre-allocated global lists
  private final List<Pose3d> allTagPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
  private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  private final List<VisionObservation> allVisionObservations = new ArrayList<>();

  // GC Optimization: Reusable lists for the current camera being processed
  private final List<Pose3d> currentTagPoses = new ArrayList<>();
  private final List<Pose3d> currentRobotPoses = new ArrayList<>();
  private final List<Pose3d> currentRobotPosesAccepted = new ArrayList<>();
  private final List<Pose3d> currentRobotPosesRejected = new ArrayList<>();

  // 2. String Optimization: Pre-computed AdvantageKit logging keys
  private final String[] processInputKeys;
  private final String[] tagPosesKeys;
  private final String[] robotPosesKeys;
  private final String[] robotPosesAcceptedKeys;
  private final String[] robotPosesRejectedKeys;
  private final String[] observationNamePrefixes;

  public Vision(VisionIO... io) {
    this.io = io;

    // Initialize arrays
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];

    this.processInputKeys = new String[io.length];
    this.tagPosesKeys = new String[io.length];
    this.robotPosesKeys = new String[io.length];
    this.robotPosesAcceptedKeys = new String[io.length];
    this.robotPosesRejectedKeys = new String[io.length];
    this.observationNamePrefixes = new String[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);

      // Pre-compute all strings to prevent loop overrun concatenation
      processInputKeys[i] = "Vision/Camera" + i;
      tagPosesKeys[i] = "Vision/Camera" + i + "/TagPoses";
      robotPosesKeys[i] = "Vision/Camera" + i + "/RobotPoses";
      robotPosesAcceptedKeys[i] = "Vision/Camera" + i + "/RobotPosesAccepted";
      robotPosesRejectedKeys[i] = "Vision/Camera" + i + "/RobotPosesRejected";
      observationNamePrefixes[i] = "Limelight" + i + " ";
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    // Clear global lists once per loop instead of making new ones
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();
    allVisionObservations.clear();

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(processInputKeys[i], inputs[i]);
    }

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Clear reusable camera-specific lists
      currentTagPoses.clear();
      currentRobotPoses.clear();
      currentRobotPosesAccepted.clear();
      currentRobotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          currentTagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                || !allowVisionMeasurements;

        currentRobotPoses.add(observation.pose());
        if (rejectPose) {
          currentRobotPosesRejected.add(observation.pose());
          continue; // Skip the rest of the processing if rejected
        } else {
          currentRobotPosesAccepted.add(observation.pose());
        }

        double distance = observation.averageTagDistance();
        double count = observation.tagCount();
        double stdDevFactor = (distance * distance) / (count * count);

        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = Double.POSITIVE_INFINITY;

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Add vision observation using cached string prefix
        allVisionObservations.add(
            new VisionObservation(
                observation.timestamp(),
                observation.pose().toPose2d(),
                new Matrix<>(VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)),
                observationNamePrefixes[cameraIndex] + observation.type().name()));
      }

      // 3. Array Optimization: Log camera-specific lists using [0] to bypass zeroing penalty
      Logger.recordOutput(tagPosesKeys[cameraIndex], currentTagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(robotPosesKeys[cameraIndex], currentRobotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          robotPosesAcceptedKeys[cameraIndex], currentRobotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          robotPosesRejectedKeys[cameraIndex], currentRobotPosesRejected.toArray(new Pose3d[0]));

      // Add to global summary lists
      allTagPoses.addAll(currentTagPoses);
      allRobotPoses.addAll(currentRobotPoses);
      allRobotPosesAccepted.addAll(currentRobotPosesAccepted);
      allRobotPosesRejected.addAll(currentRobotPosesRejected);
    }

    // Log global summary lists
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

    // Sort and submit observations
    allVisionObservations.sort(Comparator.comparingDouble(VisionObservation::timestamp));
    for (VisionObservation obs : allVisionObservations) {
      RobotState.getInstance().addVisionObservation(obs);
    }
  }
}
