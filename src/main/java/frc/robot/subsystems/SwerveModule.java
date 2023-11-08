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
    m_turnPIDController.setTolerance(1);
    m_turnPIDController.reset();

    m_moduleName = moduleName;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getAbsolutePosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getAbsolutePosition());
  }

  private Rotation2d getAbsolutePosition() {
    return Rotation2d.fromDegrees(m_turnEncoder.getAbsolutePosition());
  }

  public void setModuleState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAbsolutePosition());

    SmartDashboard.putNumber(m_moduleName+" Opt Speed", optimizedState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName+" Opt Angle", optimizedState.angle.getDegrees());
    SmartDashboard.putNumber(m_moduleName+" Absolute Pos", getAbsolutePosition().getDegrees());
    // SmartDashboard.putNumber(m_moduleName+" Unopt Speed", desiredState.speedMetersPerSecond);
    // SmartDashboard.putNumber(m_moduleName+" UnOpt Angle", desiredState.angle.getDegrees());

    m_driveMotor.set(0.8 * optimizedState.speedMetersPerSecond);

    m_turnMotor.set(m_turnPIDController.calculate(getAbsolutePosition().getRadians(), optimizedState.angle.getRadians()));
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }
}
