package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Vision {

    public static PoseEstimate getLimelightMegaTag2Pose(String camName, Pose2d currentPose, double currentYawRate) {
        LimelightHelpers.SetRobotOrientation(camName, currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);

        if (mt2 != null && Math.abs(currentYawRate) <= 720 && mt2.tagCount != 0) {
            return mt2;
        }
        return null;
    }

    public static EstimatedRobotPose getPhotonMultiTagPose(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
        if (!camera.getCameraTable().getEntry("heartbeat").exists())
            return null;

        var result = poseEstimator.update();
        if (result.isPresent()) {
            return result.get();
        }
        return null;
    }

    public static void updateAprilTags(Field2d field, PhotonCamera photonCam) {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        List<Pose2d> aprilTagPoses = new ArrayList<>();

        for (PhotonTrackedTarget target : photonCam.getLatestResult().getTargets()) {
            var pose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

            pose.ifPresent((certainPose) -> aprilTagPoses.add(certainPose.toPose2d()));
        }

        // for (int i = 0; i < aprilTagPoses.size(); i++) {
        // FieldObject2d aprilTag = field.getObject("AprilTag" + i);
        // aprilTag.setPose(aprilTagPoses.get(i));
        // }

        FieldObject2d aprilTags = field.getObject("AprilTags");
        aprilTags.setPoses(aprilTagPoses);
    }

}
