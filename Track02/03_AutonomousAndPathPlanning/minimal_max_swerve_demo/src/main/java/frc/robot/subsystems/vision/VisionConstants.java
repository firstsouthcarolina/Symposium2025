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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.simulation.VisionSystemSim;

public class VisionConstants {
  public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {}

  public static enum PoseEstimationMethod {
    MULTI_TAG,
    SINGLE_TAG,
    TRIG,
    CONSTRAINED
  }

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final Optional<VisionSystemSim> aprilTagSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("AprilTagSim"))
          : Optional.empty();

  private static final List<AprilTagCameraConfig> guidoConfigs =
      List.of(
          new AprilTagCameraConfig(
              new VisionSource(
                  "SillyCam",
                  new Transform3d(
                      new Translation3d(
                          6.0 / 100.0, // forward+
                          29.5 / 100.0, // left+
                          26.5 / 100.0), // up+
                      new Rotation3d(0, Units.degreesToRadians(-17.5), 0))),
              SimCameraConfig.THRIFTY_CAM_90));

  private static final List<AprilTagCameraConfig> riptideConfigs =
      List.of(
          // FLO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-5.671), // forward+
                          Units.inchesToMeters(11.237), // left+
                          Units.inchesToMeters(7.991)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-20.0), Units.degreesToRadians(30)))),
              SimCameraConfig.THRIFTY_CAM_80),
          // FLI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(6.354), // forward+
                          Units.inchesToMeters(11.143), // left+
                          Units.inchesToMeters(8.058)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-27.5), Units.degreesToRadians(-10)))),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(6.354), // forward+
                          Units.inchesToMeters(-11.143), // left+
                          Units.inchesToMeters(8.058)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-27.5), Units.degreesToRadians(10)))),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-5.671), // forward+
                          Units.inchesToMeters(-11.237), // left+
                          Units.inchesToMeters(7.991)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-30)))),
              SimCameraConfig.THRIFTY_CAM_80));

  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs = riptideConfigs;

  public static final double ambiguityCutoff = 0.05;
  public static final double singleTagPoseCutoffMeters = 4.0;

  public static final int noAmbiguity = -100;

  public static final Optional<ConstrainedSolvepnpParams> constrainedSolvePnpParams =
      Optional.of(new ConstrainedSolvepnpParams(true, 0));

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> noStdDevs = VecBuilder.fill(0, 0, 0);
  public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, Double.MAX_VALUE);
  public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);

  public static final Matrix<N3, N1> tagStdDevs = VecBuilder.fill(0, 0, Double.MAX_VALUE);
}
