package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefSectors extends SubsystemBase {
    private final Supplier<Pose2d> m_poseSupplier;
    private final Field2d m_field;

    boolean m_isRedAlliance = false;

    private double m_reefCenterX = 0;
    private final double m_reefCenterY = 4.037;

    private final double boundaryBoxWidth = 8;
    private final double boundaryBoxLength = 6;

    private double minX = 0;
    private double maxX = 0;
    private final double minY = m_reefCenterY - boundaryBoxWidth / 2;
    private final double maxY = m_reefCenterY + boundaryBoxWidth / 2;

    public ReefSectors(Supplier<Pose2d> poseSupplier, Field2d field) {
        m_poseSupplier = poseSupplier;
        m_field = field;
    }

    @Override
    public void periodic() {
        // Update variables constantly
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_isRedAlliance = alliance.get() == DriverStation.Alliance.Red;
            m_reefCenterX = m_isRedAlliance ? 13.054 : 4.476;
        }

        minX = m_reefCenterX - boundaryBoxLength / 2;
        maxX = m_reefCenterX + boundaryBoxLength / 2;

        // Display lines on the field
        m_field.getObject("Sectors Zone").setPoses(
                new Pose2d(minX, minY, Rotation2d.fromDegrees(0)),
                new Pose2d(maxX, minY, Rotation2d.fromDegrees(0)),
                new Pose2d(maxX, maxY, Rotation2d.fromDegrees(0)),
                new Pose2d(minX, maxY, Rotation2d.fromDegrees(0)),
                new Pose2d(minX, minY, Rotation2d.fromDegrees(0)));

        List<Pose2d> sectorLines = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            double sectorAngle = Math.toRadians(i * 60 + 30); // Rotate by 30 degrees
            double endX = m_reefCenterX + 10 * Math.cos(sectorAngle); // Extend the line to a length of 10 meters
            double endY = m_reefCenterY + 10 * Math.sin(sectorAngle);

            // Calculate intersection points with the sector zone box
            double intersectionX = endX;
            double intersectionY = endY;

            if (endX < minX) {
                intersectionX = minX;
                intersectionY = m_reefCenterY + (minX - m_reefCenterX) * Math.tan(sectorAngle);
            } else if (endX > maxX) {
                intersectionX = maxX;
                intersectionY = m_reefCenterY + (maxX - m_reefCenterX) * Math.tan(sectorAngle);
            }

            if (intersectionY < minY) {
                intersectionY = minY;
                intersectionX = m_reefCenterX + (minY - m_reefCenterY) / Math.tan(sectorAngle);
            } else if (intersectionY > maxY) {
                intersectionY = maxY;
                intersectionX = m_reefCenterX + (maxY - m_reefCenterY) / Math.tan(sectorAngle);
            }

            sectorLines.add(new Pose2d(m_reefCenterX, m_reefCenterY, new Rotation2d(sectorAngle)));
            sectorLines.add(new Pose2d(intersectionX, intersectionY, new Rotation2d(sectorAngle)));
        }
        m_field.getObject("Sector Lines").setPoses(sectorLines);

        SmartDashboard.putNumber("REEF Sector", getReefSector());
        SmartDashboard.putNumber("Target Tag ID", getNearestSectorTag());
    }

    public int getReefSector() {
        Pose2d robotPose = m_poseSupplier.get();
        double x = robotPose.getX();
        double y = robotPose.getY();
        int sector = -1;

        if (x >= minX && x <= maxX && y >= minY && y <= maxY) {
            double angle = Math.atan2(y - m_reefCenterY, x - m_reefCenterX);
            angle = Math.toDegrees(angle);
            if (angle < 0) {
                angle += 360;
            }
            // Rotate the angle by 30 degrees
            angle = (angle + 30) % 360;
            sector = (int) (angle / 60);
        }

        return sector;
    }

    public int getNearestSectorTag() {
        int sector = getReefSector();
        boolean isRedAlliance = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRedAlliance = alliance.get() == DriverStation.Alliance.Red;
        }
        int tagId;

        if (isRedAlliance) {
            if (sector == 0) {
                tagId = 7;
            } else if (sector == 1) {
                tagId = 8;
            } else if (sector == 2) {
                tagId = 9;
            } else if (sector == 3) {
                tagId = 10;
            } else if (sector == 4) {
                tagId = 11;
            } else if (sector == 5) {
                tagId = 6;
            } else {
                return -1;
            }
        } else {
            if (sector == 0) {
                tagId = 21;
            } else if (sector == 1) {
                tagId = 20;
            } else if (sector == 2) {
                tagId = 19;
            } else if (sector == 3) {
                tagId = 18;
            } else if (sector == 4) {
                tagId = 17;
            } else if (sector == 5) {
                tagId = 22;
            } else {
                return -1;
            }
        }

        return tagId;
    }
}
