// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TrackTargetAutoCmd extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_limelightTable;
    private final String m_cameraKey = "OV5647/";
    private final PIDController m_yawPIDController = new PIDController(0.01, 0, 0);
    private final PIDController m_rangePIDController = new PIDController(0.05, 0, 0);

  /** Creates a new TrackTargetAutoCmd. */
  public TrackTargetAutoCmd(DriveSubsystem driveSubsystem) {
    m_limelightTable = m_inst.getTable("photonvision");

    m_driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetYaw = m_limelightTable.getEntry(m_cameraKey+"targetYaw").getDouble(0);
    double targetArea = m_limelightTable.getEntry(m_cameraKey+"targetArea").getDouble(8);

    if (targetArea == 0) {
      targetArea = 8;
    }

    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Target Area", targetArea);

    m_driveSubsystem.drive(
        m_rangePIDController.calculate(targetArea, 8),
        0,
        -m_yawPIDController.calculate(targetYaw, 0)
        // 0
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
