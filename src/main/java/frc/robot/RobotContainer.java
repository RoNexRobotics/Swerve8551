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
import frc.robot.commands.AprilTagFollowCmd;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.ElasticNotification;
import frc.robot.util.Elastic.ElasticNotification.NotificationLevel;

public class RobotContainer {
  // Subsystems
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Controllers
  CommandJoystick m_driverJoystick = new CommandJoystick(OIConstants.kDriverControllerPort);
  // CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // Other stuff
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Elastic.sendAlert(new ElasticNotification(NotificationLevel.INFO, "Woohoo", "The robot program has started up!"));

    registerNamedCommands();
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveCommand(
    () -> -MathUtil.applyDeadband(m_driverJoystick.getY(),
    OIConstants.kDriverControllerTranslationDeadband),
    () -> -MathUtil.applyDeadband(m_driverJoystick.getX(),
    OIConstants.kDriverControllerTranslationDeadband),
    () -> -MathUtil.applyDeadband(m_driverJoystick.getZ(),
    OIConstants.kDriverControllerRotationDeadband));

    // Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveCommand(
    //     () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriverControllerDeadband),
    //     () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriverControllerDeadband),
    //     () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriverControllerDeadband));

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // m_swerveSubsystem.setDefaultCommand(new AprilTagFollowCmd(m_swerveSubsystem));

    // Setup auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("AimAndShoot", Commands.none());
    NamedCommands.registerCommand("Intake", Commands.none());
  }

  private void configureBindings() {

    m_driverJoystick.button(3).onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro, m_swerveSubsystem));

    m_driverJoystick.button(5).onTrue(new InstantCommand(m_swerveSubsystem::toggleFieldRelative, m_swerveSubsystem));

    m_driverJoystick.button(6).toggleOnTrue(new AprilTagFollowCmd(m_swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
