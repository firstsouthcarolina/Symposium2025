package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public enum TurretPosition {
  INITIAL(Rotation2d.fromDegrees(0)),
  GOAL_1(Rotation2d.fromDegrees(-90)),
  GOAL_2(Rotation2d.fromDegrees(90)),
  ;

  public final Rotation2d position;

  private TurretPosition(Rotation2d position) {
    this.position = position;
  }
}
