// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    m_driveSubsystem.drive(
      -MathUtil.applyDeadband(m_controller.getY(), 0.1),
      MathUtil.applyDeadband(m_controller.getX(), 0.1),
      MathUtil.applyDeadband(m_controller.getZ(), 0.1)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
