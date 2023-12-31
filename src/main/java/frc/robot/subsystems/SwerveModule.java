// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turnEncoder;

  private final PIDController m_turnPIDController = new PIDController(ModuleConstants.kTurnP, ModuleConstants.kTurnI, ModuleConstants.kTurnD);
  private final SlewRateLimiter m_driveLimiter = new SlewRateLimiter(0.8);

  private final String m_moduleName;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double magneticOffset, boolean driveInverted, String moduleName) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = new CANCoder(encoderID);

    // Drive motor configuration
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(driveInverted);
    // m_driveMotor.burnFlash();

    // Turn motor configuration
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setInverted(false);
    m_turnMotor.burnFlash();

    // Drive encoder configuration
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

    // Turn encoder configuration
    m_turnEncoder.configFactoryDefault();
    m_turnEncoder.configMagnetOffset(magneticOffset);
    m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turnEncoder.configSensorDirection(true);

    // Turn PID controller configuration
    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnPIDController.reset();

    m_moduleName = moduleName;
  }

  public void logStuff() {
    // Log stuff about the modules
    SmartDashboard.putNumber(m_moduleName+" Angle", getTurnAngle().getDegrees());
    SmartDashboard.putNumber(m_moduleName+" Speed", m_driveMotor.get());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  private Rotation2d getTurnAngle() {
    return Rotation2d.fromDegrees(m_turnEncoder.getAbsolutePosition());
  }

  public void setModuleState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    
    // Optimize the state so the wheel rotates the least distance possible
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getTurnAngle());

    // Set the drive speed
    m_driveMotor.set(m_driveLimiter.calculate(optimizedState.speedMetersPerSecond));

    // Set the turn angle
    m_turnMotor.set(m_turnPIDController.calculate(getTurnAngle().getRadians(), optimizedState.angle.getRadians()));
  }

  public void stop() {
    // Stop both motors completely
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }
}
