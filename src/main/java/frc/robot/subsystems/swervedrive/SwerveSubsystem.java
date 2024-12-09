// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;

  private boolean m_fieldRelative = true;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(SwerveConstants.kConfigDirectory).createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    m_swerve.setCosineCompensator(!RobotBase.isSimulation());
    m_swerve.setModuleEncoderAutoSynchronize(true, 0);
    m_swerve.setHeadingCorrection(true);

    setupPathPlanner();
  }

  @Override
  public void periodic() {

    Pose3d targetPose3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-better");
    Pose2d targetPose2d = new Pose2d(Units.metersToInches(targetPose3d.getZ()), Units.metersToInches(targetPose3d.getX()), Rotation2d.fromRadians(targetPose3d.getRotation().getY()));

    SmartDashboard.putString("Target Pose", Math.round(targetPose2d.getX()) + ", " + Math.round(targetPose2d.getY()) + " (" + Math.round(targetPose2d.getRotation().getDegrees()) + ")");

    SmartDashboard.putBoolean("CanSeeTarget", LimelightHelpers.getFiducialID("limelight-better") == 1);

    SmartDashboard.putBoolean("Field Relative", m_fieldRelative);

    if (SwerveConstants.kMegaTag2Enabled) {
      // Limelight 3G
      LimelightHelpers.SetRobotOrientation("limelight-better", m_swerve.getYaw().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-better");
      if (Math.abs(m_swerve.getGyro().getRate()) <= 720 && mt2 != null && mt2.tagCount != 0) {
        m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
    }
  }

  private void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        m_swerve::getPose, // Robot pose supplier
        m_swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        m_swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            AutoConstants.kTranslationPID, // Translation PID constants
            AutoConstants.kAnglePID, // Rotation PID constants
            m_swerve.getMaximumVelocity(), // Max module speed, in m/s
            m_swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive
                                                                          // base
                                                                          // radius
                                                                          // in
                                                                          // meters.
                                                                          // Distance
                                                                          // from
            // robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> drive(translationX, translationY, angularRotationX));
  }

  public void drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {

    if (translationX.getAsDouble() == 0 && translationY.getAsDouble() == 0 && angularRotationX.getAsDouble() == 0) {
      // Lock the robot's pose
      m_swerve.lockPose();
    } else {
      // Make the robot move
      m_swerve.drive(
          new Translation2d(
              Math.copySign(Math.pow(translationX.getAsDouble(), 2), translationX.getAsDouble())
                  * (m_swerve.getMaximumVelocity() * SwerveConstants.kSpeedPercentage),
              Math.copySign(Math.pow(translationY.getAsDouble(), 2), translationY.getAsDouble())
                  * (m_swerve.getMaximumVelocity() * SwerveConstants.kSpeedPercentage)),
          Math.copySign(Math.pow(angularRotationX.getAsDouble(), 2), angularRotationX.getAsDouble() * (m_swerve.getMaximumAngularVelocity() * SwerveConstants.kRotationSpeedPercentage)),
          // angularRotationX.getAsDouble()
          //     * (m_swerve.getMaximumAngularVelocity() * SwerveConstants.kSpeedPercentage),
          m_fieldRelative, false);
    }
  }

  public Command driveToPose(Pose2d pose) {

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        m_swerve.getMaximumVelocity() * SwerveConstants.kDriveToPoseSpeedPercentage, 4.0,
        m_swerve.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  public void addFakeVisionReading() {
    m_swerve.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public void resetOdometry() {
    m_swerve.resetOdometry(new Pose2d(8.28, 4.11, Rotation2d.fromDegrees(0)));
  }

  public void zeroGyro() {
    m_swerve.zeroGyro();
  }

  public void toggleFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
  }
}