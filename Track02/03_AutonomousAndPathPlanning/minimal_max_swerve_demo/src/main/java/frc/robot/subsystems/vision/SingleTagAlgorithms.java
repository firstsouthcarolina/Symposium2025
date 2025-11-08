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

import org.photonvision.targeting.PhotonTrackedTarget;

public class SingleTagAlgorithms {
  public static boolean isUsable(PhotonTrackedTarget target) {
    return VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).isPresent()
        && target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff
        && target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
            < VisionConstants.singleTagPoseCutoffMeters;
  }
}
