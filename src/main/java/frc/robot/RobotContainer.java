// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class RobotContainer {
  // Controllers
  private final CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Commands

  // Other stuff
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // Send a friendly message to the driver
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "YEYEYE!", "Robot program started."));

    registerNamedCommands();
    configureBindings();

    Command driveAngularVelocity = m_swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriverControllerTranslationDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriverControllerTranslationDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriverControllerRotationDeadband));

    m_swerveSubsystem.setDefaultCommand(driveAngularVelocity);

    // Send the auto chooser to the dashboard
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Test", Commands.none());
  }

  private void configureBindings() {

    m_driverController.button(3).onTrue(new InstantCommand(m_swerveSubsystem::resetGyro, m_swerveSubsystem));

    m_driverController.button(5).onTrue(new InstantCommand(m_swerveSubsystem::toggleFieldRelative, m_swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
