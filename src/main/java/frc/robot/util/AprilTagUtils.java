package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagUtils {

    private static final AprilTagFieldLayout m_aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025Reefscape);

    public static Pose3d getAprilTagPose3d(int tagId) {
        return m_aprilTagLayout.getTagPose(tagId).orElse(null);
    }
}
