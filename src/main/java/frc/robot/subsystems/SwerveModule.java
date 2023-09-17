// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turnEncoder;

  private final PIDController m_drivePIDController;
  private final PIDController m_turnPIDController;

  private final String m_moduleName;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, String moduleName) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = new CANCoder(encoderID);

    m_driveMotor.restoreFactoryDefaults();

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

    m_drivePIDController = new PIDController(ModuleConstants.kDriveD, ModuleConstants.kDriveI, ModuleConstants.kDriveD);
    m_turnPIDController = new PIDController(ModuleConstants.kTurnD, ModuleConstants.kTurnI, ModuleConstants.kTurnD);

    // m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI); // Maybe this is important? Idk?

    m_moduleName = moduleName;
  }

  public void setModuleState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_turnEncoder.getAbsolutePosition()));

    SmartDashboard.putNumber(m_moduleName+" Opt Speed", optimizedState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName+" Opt Angle", optimizedState.angle.getDegrees());
    SmartDashboard.putNumber(m_moduleName+" Unopt Speed", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName+" UnOpt Angle", desiredState.angle.getDegrees());

    m_driveMotor.set(
      m_drivePIDController.calculate(m_driveEncoder.getVelocity(), optimizedState.speedMetersPerSecond)
    );

    m_turnMotor.set(
      m_turnPIDController.calculate(m_turnEncoder.getAbsolutePosition(), optimizedState.angle.getRadians())
    );
  }
}
