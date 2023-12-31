// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleopCmd extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;

  private final CommandXboxController m_controller;

  /** Creates a new DriveWithJoystick. */
  public DriveTeleopCmd(DriveSubsystem driveSubsystem, CommandXboxController controller) {
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
    // Joystick
    // double xSpeed = -MathUtil.applyDeadband(m_controller.getY(), OperatorConstants.kDriverControllerDeadband);
    // double ySpeed = MathUtil.applyDeadband(m_controller.getX(), OperatorConstants.kDriverControllerDeadband);
    // double rotSpeed = MathUtil.applyDeadband(m_controller.getZ(), OperatorConstants.kDriverControllerDeadband);

    // Xbox Controller
    double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.kDriverControllerDeadband) * 0.3;
    double ySpeed = MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.kDriverControllerDeadband) * 0.3;
    double rotSpeed = MathUtil.applyDeadband(m_controller.getRightX(), OperatorConstants.kDriverControllerDeadband) * 0.4;

    if (m_controller.rightBumper().getAsBoolean()) {
      m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
    } else {
      m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed);
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
