// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TestAutoCmd extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PIDController m_xPIDController = new PIDController(0.005, 0, 0);

  /** Creates a new DriveWithJoystick. */
  public TestAutoCmd(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;

    addRequirements(driveSubsystem, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(
      0,
      -m_xPIDController.calculate(m_limelightSubsystem.getTargetX(), 165),
      0
      );
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
