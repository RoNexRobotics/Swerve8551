// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
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

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    getHeading(),
    new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition(),
    });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // DriveSubsystem Logging
    Logger.getInstance().recordOutput("RobotHeadingDegrees", getHeading().getDegrees());
    Logger.getInstance().recordOutput("RobotPose", m_odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    // Update odometry
    m_odometry.update(
      getHeading(),
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_rearLeftModule.getPosition(),
        m_rearRightModule.getPosition(),
      });
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    Logger.getInstance().recordOutput("SwerveModuleStates", states);

    m_frontLeftModule.setModuleState(states[0]);
    // m_frontRightModule.setModuleState(states[1]);
    m_rearLeftModule.setModuleState(states[2]);
    // m_rearRightModule.setModuleState(states[3]);
  }

  public void stopModules() {
    m_frontLeftModule.stop();
    m_frontRightModule.stop();
    m_rearLeftModule.stop();
    m_rearRightModule.stop();
  }
}
