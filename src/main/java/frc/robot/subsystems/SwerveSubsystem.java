// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;
  private boolean m_fieldRelative = true;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    // Set swerve telemetry verbosity
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(SwerveConstants.kSwerveConfigurationDirectory)
          .createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Configure swerve drive
    m_swerve.setHeadingCorrection(true);
    m_swerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    m_swerve.setAngularVelocityCompensation(true, true, 0.1);
    m_swerve.setModuleEncoderAutoSynchronize(true, 1);

    setupPathPlanner();
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Field Relative", m_fieldRelative);
    SmartDashboard.putBoolean("MegaTag2", SwerveConstants.kMegaTag2Enabled);

    if (SwerveConstants.kMegaTag2Enabled) {
      // Limelight 3G
      LimelightHelpers.SetRobotOrientation("limelight-better",
          m_swerve.getYaw().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-better");
      if (Math.abs(m_swerve.getGyro().getYawAngularVelocity().magnitude()) <= 720
          && mt2 != null && mt2.tagCount != 0) {
        m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }

      if (mt2 != null) {
        AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        String[] poses = new String[mt2.rawFiducials.length];
        for (int i = 0; i < mt2.rawFiducials.length; i++) {
          RawFiducial rawFiducial = mt2.rawFiducials[i];
          poses[i] = aprilTagLayout.getTagPose(rawFiducial.id).orElse(null).toString();
        }
        SmartDashboard.putStringArray("AprilTag Poses", poses);
      } else {
        SmartDashboard.putStringArray("AprilTag Poses", new String[0]);
      }

    }
  }

  private void setupPathPlanner() {
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          m_swerve::getPose,
          m_swerve::resetOdometry,
          m_swerve::getRobotVelocity,
          (speeds, feedforwards) -> m_swerve.drive(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotation) {
    return run(() -> {
      m_swerve.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * m_swerve.getMaximumChassisVelocity(),
          translationY.getAsDouble() * m_swerve.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotation.getAsDouble(), 3) * m_swerve.getMaximumChassisAngularVelocity(), m_fieldRelative,
          false);
    });
  }

  public void toggleFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
  }

  public void resetGyro() {
    m_swerve.zeroGyro();
  }
}
