// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeftModule = new SwerveModule(
    DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftTurnID, DriveConstants.kFrontLeftEncoderID, "FL");
  private final SwerveModule m_frontRightModule = new SwerveModule(
    DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightTurnID, DriveConstants.kFrontRightEncoderID, "FR");
  private final SwerveModule m_rearLeftModule = new SwerveModule(
    DriveConstants.kRearLeftDriveID, DriveConstants.kRearLeftTurnID, DriveConstants.kRearLeftEncoderID, "RL");
  private final SwerveModule m_rearRightModule = new SwerveModule(
    DriveConstants.kRearRightDriveID, DriveConstants.kRearRightTurnID, DriveConstants.kRearRightEncoderID, "RR");

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, Rotation2d.fromDegrees(0));

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.getInstance().recordOutput("SwerveModuleStates", states);

    m_frontLeftModule.setModuleState(states[1]);
    m_frontRightModule.setModuleState(states[0]);
    m_rearLeftModule.setModuleState(states[3]);
    m_rearRightModule.setModuleState(states[2]);
  }
}
