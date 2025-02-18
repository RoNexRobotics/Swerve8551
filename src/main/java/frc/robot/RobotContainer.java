// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignWithNearestSectorTag;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class RobotContainer {
  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

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
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriverControllerTranslationDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriverControllerTranslationDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriverControllerRotationDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriverControllerRotationDeadband))
        .beforeStarting(new InstantCommand(() -> m_swerveSubsystem.setCommandedHeading(), m_swerveSubsystem));

    m_swerveSubsystem.setDefaultCommand(driveAngularVelocity);

    // Send the auto chooser to the dashboard
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Test", Commands.none());
    NamedCommands.registerCommand("AlignWithNearestSectorTag", new AlignWithNearestSectorTag(m_swerveSubsystem,
        new Transform2d(
            Units.inchesToMeters(24),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(180))));
  }

  private void configureBindings() {

    m_driverController.leftBumper().onTrue(new InstantCommand(m_swerveSubsystem::resetGyro, m_swerveSubsystem));

    m_driverController.rightBumper().onTrue(new InstantCommand(m_swerveSubsystem::resetPose, m_swerveSubsystem));

    m_driverController.x()
        .whileTrue(m_swerveSubsystem.alignWithAprilTag(18,
            new Transform2d(Units.inchesToMeters(48), Units.inchesToMeters(0), Rotation2d.fromDegrees(0))));

    m_driverController.y().whileTrue(new AlignWithNearestSectorTag(m_swerveSubsystem, new Transform2d(
        Units.inchesToMeters(24),
        Units.inchesToMeters(0),
        Rotation2d.fromDegrees(180))));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
