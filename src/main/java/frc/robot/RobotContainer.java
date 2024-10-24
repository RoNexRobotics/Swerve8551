// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveTeleopCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // Subsystems
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Controllers
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Commands
  DriveTeleopCmd m_driveTeleopCmd = new DriveTeleopCmd(m_swerveSubsystem, m_driverController);

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_driveTeleopCmd);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightBumper().onTrue(new InstantCommand(m_swerveSubsystem::resetHeading, m_swerveSubsystem));

    m_driverController.y().onTrue(new InstantCommand(m_swerveSubsystem::resetPose, m_swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
