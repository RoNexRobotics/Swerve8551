// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithJoystickCmd extends CommandBase {
  DriveSubsystem m_driveSubsystem;

  Joystick m_controller;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystickCmd(DriveSubsystem driveSubsystem, Joystick controller) {
    m_driveSubsystem = driveSubsystem;
    m_controller = controller;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -MathUtil.applyDeadband(m_controller.getY(), OperatorConstants.kDriverControllerDeadband);
    double ySpeed = MathUtil.applyDeadband(m_controller.getX(), OperatorConstants.kDriverControllerDeadband);
    double rotSpeed = MathUtil.applyDeadband(m_controller.getZ(), OperatorConstants.kDriverControllerDeadband);

    // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, m_driveSubsystem.getHeading()); // TODO: This cool stuff
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    m_driveSubsystem.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
