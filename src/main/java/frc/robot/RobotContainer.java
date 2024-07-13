// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
  // Subsystems
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // Other stuff
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerNamedCommands();
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriverControllerDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriverControllerDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriverControllerDeadband));

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Setup auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("SayHi", new InstantCommand(() -> System.out.println("Hiii!!")));
  }

  private void configureBindings() {
    m_driverController.x()
        .whileTrue(m_swerveSubsystem.driveToPose(new Pose2d(1.41, 5.53, Rotation2d.fromDegrees(180))));
    m_driverController.b()
        .whileTrue(m_swerveSubsystem.driveToPose(new Pose2d(14.96, 1.22, Rotation2d.fromDegrees(-60.26))));
    m_driverController.y().whileTrue(Commands.run(m_swerveSubsystem::addFakeVisionReading));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
