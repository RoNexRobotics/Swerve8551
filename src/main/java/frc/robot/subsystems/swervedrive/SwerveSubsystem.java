// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
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

    m_swerve.setCosineCompensator(!RobotBase.isSimulation());
  }

  @Override
  public void periodic() {}

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    // m_swerve.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      // Make the robot move
      m_swerve.driveFieldOriented(m_swerve.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      m_swerve.getOdometryHeading().getRadians(),
                                                                      m_swerve.getMaximumVelocity()));
    });
  }
}
