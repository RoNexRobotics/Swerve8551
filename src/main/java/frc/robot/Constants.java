// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class SwerveConstants {
    // Constants that differ among all swerve modules

    // Motor CAN IDs
    public static final int kFrontLeftDriveID = 1;
    public static final int kFrontRightDriveID = 3;
    public static final int kRearLeftDriveID = 5;
    public static final int kRearRightDriveID = 7;
    public static final int kFrontLeftAngleID = 2;
    public static final int kFrontRightAngleID = 4;
    public static final int kRearLeftAngleID = 6;
    public static final int kRearRightAngleID = 8;

    // Absolute encoder CAN IDs
    public static final int kFrontLeftEncoderID = 9;
    public static final int kFrontRightEncoderID = 10;
    public static final int kRearLeftEncoderID = 11;
    public static final int kRearRightEncoderID = 12;

    // Magnetic offsets
    public static final double kFrontLeftMagneticOffset = 0;
    public static final double kFrontRightMagneticOffset = 0;
    public static final double kRearLeftMagneticOffset = 0;
    public static final double kRearRightMagneticOffset = 0;

    // Drive motor inversions
    public static final boolean kFrontLeftDriveInverted = false;
    public static final boolean kFrontRightDriveInverted = false;
    public static final boolean kRearLeftDriveInverted = false;
    public static final boolean kRearRightDriveInverted = false;

    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
  }

  public static class ModuleConstants {
    // Constants that are the same among all swerve modules

    public static final double kAngleP = 0.4;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;

    // Encoder conversion factors
    public static final double kDriveEncoderPositionFactor = 0.04336763771803301;
    public static final double kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0;
    public static final double kAngleEncoderPositionFactor = 26.92313445114199;
    public static final double kAngleEncoderVelocityFactor = kAngleEncoderPositionFactor / 60.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.05;
  }
}
