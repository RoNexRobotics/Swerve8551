// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turnEncoder;
  private final RelativeEncoder m_relTurnEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kDriveD, ModuleConstants.kDriveI, ModuleConstants.kDriveD);
  private final SparkMaxPIDController m_turnPIDController;

  private final String m_moduleName;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, String moduleName) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = new CANCoder(encoderID);
    m_relTurnEncoder = m_turnMotor.getEncoder();

    m_turnPIDController = m_turnMotor.getPIDController();

    m_turnPIDController.setFeedbackDevice(m_relTurnEncoder);

    m_turnPIDController.setP(0.2);
    m_turnPIDController.setI(0);
    m_turnPIDController.setD(0.7);

    m_driveMotor.restoreFactoryDefaults();

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

    // m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI); // Maybe this is important? Idk?

    m_moduleName = moduleName;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turnEncoder.getAbsolutePosition())); // TODO: Check the units
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromDegrees(m_turnEncoder.getAbsolutePosition()));
  }

  public void setModuleState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(m_turnEncoder.getAbsolutePosition()));

    SmartDashboard.putNumber(m_moduleName+" Opt Speed", optimizedState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName+" Opt Angle", optimizedState.angle.getDegrees());
    SmartDashboard.putNumber(m_moduleName+" Unopt Speed", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName+" UnOpt Angle", desiredState.angle.getDegrees());

    // m_driveMotor.set(
    //   m_drivePIDController.calculate(m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond)
    // );
    m_driveMotor.set(0);

    m_relTurnEncoder.setPosition(Units.degreesToRadians(m_turnEncoder.getAbsolutePosition()));

    m_turnPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }
}
