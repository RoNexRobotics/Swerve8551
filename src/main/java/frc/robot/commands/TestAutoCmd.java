// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TestAutoCmd extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  Timer m_timer = new Timer();

  /** Creates a new DriveWithJoystick. */
  public TestAutoCmd(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() < 5) {
        m_driveSubsystem.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.4, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.4, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.4, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.4, Rotation2d.fromDegrees(0))
        });
    } else {
        m_driveSubsystem.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        });
    }
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
