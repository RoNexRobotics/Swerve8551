// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPoseCmd extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController m_xPIDController = new PIDController(2, 0, 0.1);
  private final PIDController m_yPIDController = new PIDController(2, 0, 0.1);
  private final PIDController m_rotPIDController = new PIDController(2, 0, 0.1);

  private final Pose2d m_targetPose = new Pose2d(
    new Translation2d(1, 0),
    Rotation2d.fromDegrees(45)
  );

  /** Creates a new GoToPoseCmd. */
  public GoToPoseCmd(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;

    m_rotPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = m_driveSubsystem.getPose();

      m_driveSubsystem.drive(
        m_xPIDController.calculate(pose.getX(), m_targetPose.getX()),
        -m_yPIDController.calculate(pose.getY(), m_targetPose.getY()),
        -m_rotPIDController.calculate(pose.getRotation().getRadians(), m_targetPose.getRotation().getRadians()),
        true
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
