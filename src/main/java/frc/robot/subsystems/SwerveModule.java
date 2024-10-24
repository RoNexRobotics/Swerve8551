package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_angleMotor;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_angleEncoder;
    // private final PIDController m_drivePIDController = new
    // PIDController(ModuleConstants.kDriv, 0, 0)
    private final PIDController m_anglePIDController = new PIDController(ModuleConstants.kAngleP,
            ModuleConstants.kAngleI, ModuleConstants.kAngleD);
    private final CANcoder m_absEncoder;

    public SwerveModule(int driveMotorId, int angleMotorId, int absEncoderId, double magneticOffset,
            boolean driveInverted) {
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        m_angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_angleEncoder = m_driveMotor.getEncoder();
        m_absEncoder = new CANcoder(absEncoderId);

        // Drive motor configuration
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setInverted(driveInverted);

        // Angle motor configuration
        m_angleMotor.restoreFactoryDefaults();
        m_angleMotor.setInverted(false);

        // Drive encoder configuration
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

        // Angle encoder configuration
        m_angleEncoder.setPositionConversionFactor(ModuleConstants.kAngleEncoderPositionFactor);
        m_angleEncoder.setVelocityConversionFactor(ModuleConstants.kAngleEncoderVelocityFactor);

        // Angle PID controller configuration
        m_anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_anglePIDController.reset();

        // Absolute encoder configuration
        CANcoderConfiguration absEncoderConfig = new CANcoderConfiguration();
        absEncoderConfig.MagnetSensor.MagnetOffset = magneticOffset;
        absEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        absEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        m_absEncoder.getConfigurator().apply(absEncoderConfig);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(m_absEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setModuleState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize the state so the wheel rotates the least distance possible
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAngle());

        // Set the drive speed
        // m_driveMotor.set(m_driveLimiter.calculate(optimizedState.speedMetersPerSecond));
        m_driveMotor.set(optimizedState.speedMetersPerSecond);

        // Set the turn angle
        m_angleMotor
                .set(m_anglePIDController.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));
    }

    public void stop() {
        // Stop both motors completely
        m_driveMotor.stopMotor();
        m_angleMotor.stopMotor();
    }
}
