// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean kReplayMode = false;
  
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.05;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class HardwareConstants {
    public static final int kPowerDistributionId = 0;
    public static final ModuleType kPowerDistributionModuleType = ModuleType.kCTRE;
  }

  public static final class SwerveConstants {
    public static final File kConfigDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    public static final double kMaxSpeed = Units.feetToMeters(14.5);
  }
}
