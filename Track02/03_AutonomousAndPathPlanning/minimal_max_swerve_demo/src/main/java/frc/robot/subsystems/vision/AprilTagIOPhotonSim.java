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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class AprilTagIOPhotonSim extends AprilTagIOPhoton {
  private final PhotonCameraSim cameraSim;

  public AprilTagIOPhotonSim(
      VisionSource source,
      List<AprilTagStruct> constrainedTargets,
      Supplier<Rotation2d> headingSupplier,
      SimCameraConfig config) {
    super(source, constrainedTargets, headingSupplier);

    SimCameraProperties props = config.apply(new SimCameraProperties());

    cameraSim = new PhotonCameraSim(camera, props, VisionConstants.fieldLayout);

    cameraSim.enableDrawWireframe(true);
    cameraSim.setMaxSightRange(10.0);
    cameraSim.setWireframeResolution(1);

    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> aprilTagSim.addCamera(cameraSim, source.robotToCamera()));
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    super.updateInputs(inputs);

    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> {
          FieldObject2d visionEstimation =
              aprilTagSim.getDebugField().getObject("VisionEstimation");

          if (inputs.validPoseObservations.length != 0) {
            visionEstimation.setPoses(inputs.validPoseObservations[0].robotPose().toPose2d());
          } else {
            visionEstimation.setPoses();
          }
        });
  }
}
