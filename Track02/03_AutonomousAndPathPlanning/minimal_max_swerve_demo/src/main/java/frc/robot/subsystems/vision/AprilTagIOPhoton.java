// Copyright 2025 FRC 4451
// http://github.com/frc4451
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimationMethod;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  protected final PhotonCamera camera;

  /** This should compute our global pose we trust */
  private final PhotonPoseEstimator globalEstimator;

  public AprilTagIOPhoton(
      VisionSource source,
      List<AprilTagStruct> trigConstrainedTargets,
      Supplier<Rotation2d> headingSupplier) {
    camera = new PhotonCamera(source.name());

    globalEstimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());

    globalEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    // List<PoseObservation> allLocalizedPoseObservations = new ArrayList<>();
    // List<PhotonTrackedTarget> allTargets = List.of();
    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      // Detected Corners
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (SingleTagAlgorithms.isUsable(target)) {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(validCorners::add);

          validIds.add(target.getFiducialId());

          validAprilTagPoses.add(
              VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get());
        } else {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(rejectedCorners::add);

          rejectedIds.add(target.getFiducialId());

          if (target.getFiducialId() != -1) {
            VisionConstants.fieldLayout
                .getTagPose(target.getFiducialId())
                .ifPresent(rejectedAprilTagPoses::add);
          }
        }
      }

      // Global Pose Estimation
      Optional<EstimatedRobotPose> maybeEstimatedPose = globalEstimator.update(result);

      if (!maybeEstimatedPose.isPresent()) {
        continue;
      }

      EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

      if (result.getMultiTagResult().isPresent()) {
        MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

        Pose3d pose = estimatedPose.estimatedPose;
        Matrix<N3, N1> stdDevs =
            AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());

        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                multiTagResult.estimatedPose.ambiguity,
                // multiTagResult.fiducialIDsUsed.stream().mapToInt(id -> id).toArray()
                VisionConstants.noAmbiguity,
                stdDevs,
                PoseEstimationMethod.MULTI_TAG);

        validPoseObservations.add(observation);
        validPoses.add(observation.robotPose());

        for (PhotonTrackedTarget target : result.getTargets()) {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(validCorners::add);

          validIds.add(target.getFiducialId());

          validAprilTagPoses.add(
              VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get());
        }
      } else if (!result.getTargets().isEmpty()) {
        PhotonTrackedTarget target = result.getTargets().get(0);

        Pose3d pose = estimatedPose.estimatedPose;
        Matrix<N3, N1> stdDevs =
            AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());
        PoseObservation observation =
            new PoseObservation(
                pose,
                estimatedPose.timestampSeconds,
                target.poseAmbiguity,
                // new int[] {target.fiducialId}
                target.fiducialId,
                stdDevs,
                PoseEstimationMethod.SINGLE_TAG);

        if (SingleTagAlgorithms.isUsable(target)) {
          validPoseObservations.add(observation);
          validPoses.add(observation.robotPose());
        } else {
          rejectedPoseObservations.add(observation);
          rejectedPoses.add(observation.robotPose());
        }
      }
    }

    inputs.connected = camera.isConnected();

    inputs.validCorners = validCorners.toArray(Translation2d[]::new);
    inputs.rejectedCorners = rejectedCorners.toArray(Translation2d[]::new);

    inputs.validIds = validIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.rejectedIds = rejectedIds.stream().mapToInt(Integer::intValue).toArray();

    inputs.validPoseObservations = validPoseObservations.toArray(PoseObservation[]::new);
    inputs.rejectedPoseObservations = rejectedPoseObservations.toArray(PoseObservation[]::new);

    inputs.validPoses = validPoses.toArray(Pose3d[]::new);
    inputs.rejectedPoses = rejectedPoses.toArray(Pose3d[]::new);

    inputs.validAprilTagPoses = validAprilTagPoses.toArray(Pose3d[]::new);
    inputs.rejectedAprilTagPoses = rejectedAprilTagPoses.toArray(Pose3d[]::new);
  }
}
