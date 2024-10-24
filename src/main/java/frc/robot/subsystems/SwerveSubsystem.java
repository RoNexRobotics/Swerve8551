// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeftModule = new SwerveModule(
      SwerveConstants.kFrontLeftDriveID,
      SwerveConstants.kFrontLeftAngleID, SwerveConstants.kFrontLeftEncoderID, SwerveConstants.kFrontLeftMagneticOffset,
      SwerveConstants.kFrontLeftDriveInverted);
  private final SwerveModule m_frontRightModule = new SwerveModule(
      SwerveConstants.kFrontRightDriveID,
      SwerveConstants.kFrontRightAngleID, SwerveConstants.kFrontRightEncoderID,
      SwerveConstants.kFrontRightMagneticOffset, SwerveConstants.kFrontRightDriveInverted);
  private final SwerveModule m_rearLeftModule = new SwerveModule(
      SwerveConstants.kRearLeftDriveID,
      SwerveConstants.kRearLeftAngleID, SwerveConstants.kRearLeftEncoderID, SwerveConstants.kRearLeftMagneticOffset,
      SwerveConstants.kRearLeftDriveInverted);
  private final SwerveModule m_rearRightModule = new SwerveModule(
      SwerveConstants.kRearRightDriveID,
      SwerveConstants.kRearRightAngleID, SwerveConstants.kRearRightEncoderID, SwerveConstants.kRearRightMagneticOffset,
      SwerveConstants.kRearRightDriveInverted);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, getHeading(),
      new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_rearLeftModule.getPosition(),
          m_rearRightModule.getPosition(),
      });

  private final Field2d m_field = new Field2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
  }

  @Override
  public void periodic() {
    m_field.setRobotPose(getPose());
    SmartDashboard.putData(m_field);

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

  public Rotation2d getHeading() {
    // Get the gyro heading (inverted because we want positive clockwise rotation)
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void resetHeading() {
    // Reset the gyro heading
    m_gyro.reset();
  }

  public Pose2d getPose() {
    // Get the odometry pose
    return new Pose2d(
        new Translation2d(m_odometry.getPoseMeters().getX(), -m_odometry.getPoseMeters().getY()),
        m_odometry.getPoseMeters().getRotation());
  }

  public void resetPose() {
    // Reset the odemetry pose
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(0),
        new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_rearLeftModule.getPosition(),
            m_rearRightModule.getPosition(),
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed) {
    drive(xSpeed, ySpeed, rotSpeed, false);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    // Convert controller inputs to chassis speeds
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

    if (fieldRelative) {
      // If field-relative mode, convert to field-relative chassis speeds
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
          Rotation2d.fromRadians(-getHeading().getRadians()));
    }

    // Convert chassis speeds to swerve module states
    SwerveModuleState[] states = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Set the module states
    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    // Reduce the wheel speeds to attainable values
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);

    // Set each module state
    m_frontLeftModule.setModuleState(states[0]);
    m_frontRightModule.setModuleState(states[1]);
    m_rearLeftModule.setModuleState(states[2]);
    m_rearRightModule.setModuleState(states[3]);
  }

  public void stopModules() {
    // Stop each module completely
    m_frontLeftModule.stop();
    m_frontRightModule.stop();
    m_rearLeftModule.stop();
    m_rearRightModule.stop();
  }
}
