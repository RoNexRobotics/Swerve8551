// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable table;
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("photonvision");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target Area", table.getEntry("OV5647/targetArea").getDouble(2.75));
    SmartDashboard.putNumber("Target X", table.getEntry("OV5647/targetPixelsX").getDouble(165));
    // 2.75 target area = 1 meter approx
  }

  public double getTargetArea() {
    return table.getEntry("OV5647/targetArea").getDouble(2.75);
  }

  public double getTargetX() {
    return table.getEntry("OV5647/targetPixelsX").getDouble(165);
  }
}
