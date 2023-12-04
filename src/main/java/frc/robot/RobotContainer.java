// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveTeleopCmd;
import frc.robot.commands.GoToPoseCmd;
import frc.robot.commands.TrackTargetAutoCmd;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Controllers
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Commands
  DriveTeleopCmd m_driveTeleopCmd = new DriveTeleopCmd(m_driveSubsystem, m_driverController);
  TrackTargetAutoCmd m_trackTargetAutoCmd = new TrackTargetAutoCmd(m_driveSubsystem);
  GoToPoseCmd m_goToPoseCmd = new GoToPoseCmd(m_driveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(m_driveTeleopCmd);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.rightBumper().onTrue(new InstantCommand(m_driveSubsystem::resetHeading, m_driveSubsystem));

    m_driverController.y().onTrue(new InstantCommand(m_driveSubsystem::resetPose, m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_trackTargetAutoCmd;
  }
}
