// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.util.CommandCustomXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandCustomXboxController m_driverController = new CommandCustomXboxController(
      ControllerConstants.kDriverControllerPort);

  private final Turret turret = new Turret();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Example 1
    m_driverController.a()
        .whileTrue(
            Commands.sequence(
                turret.setGoalAndWaitCommand(TurretPosition.GOAL_1.position),
                Commands.waitSeconds(0.5),
                turret.setGoalAndWaitCommand(TurretPosition.GOAL_2.position)))
        .onFalse(turret.setGoalCommand(TurretPosition.INITIAL.position));

    // Example 2
    // Go to pressed, go back to initial position if in conflict
    m_driverController.leftBumper().and(m_driverController.rightBumper().negate())
        .onTrue(turret.setGoalCommand(TurretPosition.GOAL_1.position))
        .onFalse(turret.setGoalCommand(TurretPosition.INITIAL.position));

    m_driverController.rightBumper().and(m_driverController.leftBumper().negate())
        .onTrue(turret.setGoalCommand(TurretPosition.GOAL_2.position))
        .onFalse(turret.setGoalCommand(TurretPosition.INITIAL.position));
  }

  private void configureDefaultCommands() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
