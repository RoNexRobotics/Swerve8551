// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  
  public static class DriveConstants {
    // Constants that are not the same across all swerve modules

    // Motor CAN IDs
    public static final int kFrontLeftDriveID = 1;
    public static final int kFrontRightDriveID = 3;
    public static final int kRearLeftDriveID = 5;
    public static final int kRearRightDriveID = 7;
    public static final int kFrontLeftTurnID = 2;
    public static final int kFrontRightTurnID = 4;
    public static final int kRearLeftTurnID = 6;
    public static final int kRearRightTurnID = 8;

    // Encoder CAN IDs
    public static final int kFrontLeftEncoderID = 9;
    public static final int kFrontRightEncoderID = 10;
    public static final int kRearLeftEncoderID = 11;
    public static final int kRearRightEncoderID = 12;

    // Chassis configuration

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(28.25); // TODO: Check these
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.25); // TODO: Check these

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Rear left
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Rear right
  }

  public static class ModuleConstants {
    // Constants that are the same across all swerve modules

    // PID Values
    public static final double kDriveP = 0.6; // TODO: Tune these
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    public static final double kTurnP = 0.6;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    // Motor conversion factors
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // TODO: Check all this
    public static final double kDrivingMotorReduction = 7.36 / 1;

    public static final double kDriveEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
    public static final double kDriveEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
