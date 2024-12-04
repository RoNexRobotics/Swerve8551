// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

public class AprilTagFollowCmd extends Command {
  private SwerveSubsystem m_swerveSubsystem;

  private PIDController m_xController = new PIDController(0.05, 0, 0);
  private PIDController m_yController = new PIDController(0.01, 0, 0);
  private PIDController m_rotController = new PIDController(0.005, 0, 0);

  private double xSpeed, ySpeed, rotSpeed;

  /** Creates a new AprilTagAlign. */
  public AprilTagFollowCmd(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fiducialId = LimelightHelpers.getFiducialID("limelight-better");
    Pose3d targetPose3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-better");
    Pose2d targetPose2d = new Pose2d(Units.metersToInches(targetPose3d.getZ()), Units.metersToInches(targetPose3d.getX()), Rotation2d.fromRadians(targetPose3d.getRotation().getY()));

    Pose2d goal = new Pose2d(36, 0, Rotation2d.fromDegrees(0));

    xSpeed = m_xController.calculate(targetPose2d.getX(), goal.getX());
    ySpeed = m_yController.calculate(targetPose2d.getY(), goal.getY());
    rotSpeed = m_rotController.calculate(targetPose2d.getRotation().getDegrees(), goal.getRotation().getDegrees());

    if (Math.abs(m_xController.getPositionError()) <= 2) {
      xSpeed = 0;
    }

    if (Math.abs(m_yController.getPositionError()) <= 1) {
      ySpeed = 0;
    }

    if (Math.abs(m_rotController.getPositionError()) <= 3) {
      rotSpeed = 0;
    }

    if (fiducialId != 1) {
      xSpeed = 0;
      ySpeed = 0;
      rotSpeed = 0;
    }

    m_swerveSubsystem.drive(
      () -> -xSpeed,
      // () -> ySpeed,
      () -> 0,
      // () -> rotSpeed
      () -> ySpeed
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
