// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Drive extends SubsystemBase {
  private final SparkMax frontLeft = new SparkMax(DriveConstants.kFrontLeftId, DriveConstants.kMotorType);
  private final SparkMax frontRight = new SparkMax(DriveConstants.kFrontRightId, DriveConstants.kMotorType);
  private final SparkMax backLeft = new SparkMax(DriveConstants.kBackLeftId, DriveConstants.kMotorType);
  private final SparkMax backRight = new SparkMax(DriveConstants.kBackRightId, DriveConstants.kMotorType);

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  public Drive() {
    SparkBaseConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    
    // The time in seconds to reach maximum output for open loop voltage control.
    // This controls your acceleration, preventing tippiness and wheel slippage.
    config.openLoopRampRate(DriveConstants.kOpenLoopRampRateSeconds);

    // Inversions should be changed based off your robot's needs
    config.inverted(false);
    SparkUtil.tryUntilOk(
        frontRight,
        5,
        () -> frontRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        backRight,
        5,
        () -> backRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    config.inverted(true);
    SparkUtil.tryUntilOk(
        frontLeft,
        5,
        () -> frontLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        backLeft,
        5,
        () -> backLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Method to run drive train in velocity control mode using curve drive.
   * Curve sets the turn proportional to the forward input,
   * thus when forward input is zero, turn output is also zero.
   *
   * However this will fallback to arcade drive turning when forward input is zero.
   * 
   * @param forwardSpeed The robot's speed along the X axis [-1.0..1.0].
   *                     Forward is positive.
   * @param turnSpeed    The normalized curvature [-1.0..1.0].
   *                     Counterclockwise is positive.
   */
  public Command curveDrive(DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
    return Commands.run(() -> {
      drive.curvatureDrive(
          forwardSpeed.getAsDouble(),
          turnSpeed.getAsDouble(),
          true);
    }, this);
  }

  /**
   * Method to run the drive train in velocity control mode using arcade drive.
   * Sets the speed based on the forward input, and adjusts the differential of the sides based
   * on the turning input to rotate the robot
   * 
   * @param forwardSpeed The robot's speed along the robot-relative X axis [-1.0..1.0].
   *                     Forward is positive.
   * @param turnSpeed    The normalized curvature [-1.0..1.0].
   *                     Counterclockwise is positive.
   */
  public Command arcadeDrive(DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
    return Commands.run(() -> {
      drive.arcadeDrive(
          forwardSpeed.getAsDouble(),
          turnSpeed.getAsDouble(),
          false);
    }, this);
  }
}
