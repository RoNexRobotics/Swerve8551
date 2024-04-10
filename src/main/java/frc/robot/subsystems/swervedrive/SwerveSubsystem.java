// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(11.3142);
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 4.59);
    System.out.println("--- Conversion Factors ---");
    System.out.println("Angle: " + angleConversionFactor);
    System.out.println("Drive: " + driveConversionFactor);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(SwerveConstants.kConfigDirectory).createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() {
    logStuff();
  }

  private void logStuff() {
    // Log fields
    Logger.recordOutput("SwervePose", m_swerve.getPose());
    Logger.recordOutput("SwerveStates", m_swerve.getStates());
    Logger.recordOutput("SwervePositions", m_swerve.getModulePositions());
    Logger.recordOutput("SwerveModulePoses", m_swerve.getSwerveModulePoses(m_swerve.getPose()));
  }
}
