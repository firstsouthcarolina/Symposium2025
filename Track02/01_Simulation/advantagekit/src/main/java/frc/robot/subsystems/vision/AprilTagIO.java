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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean connected = false;

    // public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

    public Translation2d[] validCorners = new Translation2d[0];
    public Translation2d[] rejectedCorners = new Translation2d[0];

    public int[] validIds = new int[0];
    public int[] rejectedIds = new int[0];

    public Pose3d[] validAprilTagPoses = new Pose3d[0];
    public Pose3d[] rejectedAprilTagPoses = new Pose3d[0];

    public PoseObservation[] validPoseObservations = new PoseObservation[0];
    public PoseObservation[] rejectedPoseObservations = new PoseObservation[0];

    public Pose3d[] validPoses = new Pose3d[0];
    public Pose3d[] rejectedPoses = new Pose3d[0];
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
