package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private final AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        camera = new PhotonCamera("GSC_BLACK");

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
    }

    // Get raw vision result with timestamp for fusion into odometry
    public Optional<PhotonPipelineResult> getLatestVisionResult() {
        return latestResult != null && latestResult.hasTargets() 
            ? Optional.of(latestResult) 
            : Optional.empty();
    }

    // Get the pose of the best detected AprilTag
    public Optional<Pose2d> getBestTagPose() {
        var resultOpt = getLatestVisionResult();
        if (resultOpt.isPresent()) {
            PhotonPipelineResult result = resultOpt.get();
            int bestTagId = result.getBestTarget().getFiducialId();
            return fieldLayout.getTagPose(bestTagId).isPresent() 
                ? Optional.of(fieldLayout.getTagPose(bestTagId).get().toPose2d()) 
                : Optional.empty();
        }
        return Optional.empty();
    }

    // Get the ID of the best detected tag (for reference or logging)
    public Optional<Integer> getBestTagId() {
        var resultOpt = getLatestVisionResult();
        return resultOpt.isPresent() 
            ? Optional.of(resultOpt.get().getBestTarget().getFiducialId()) 
            : Optional.empty();
    }
}

/* ----------------------------------------------------- If getting robot pose from photon vision camera -------------------------------------------------*/
// public class VisionSubsystemV2 extends SubsystemBase {
//     private final PhotonCamera camera;
//     private final PhotonPoseEstimator poseEstimator;
//     private PhotonPipelineResult latestResult;
//     private final AprilTagFieldLayout fieldLayout;

//     public VisionSubsystemV2() {
//         camera = new PhotonCamera("GSC_BLACK");

//         Transform3d robotToCam = new Transform3d(
//             new Translation3d(0.5, 0.0, 0.5), // x, y, z in meters from robot center
//             new Rotation3d(0, Math.toRadians(-30), 0) // roll, pitch, yaw
//         );

//         fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

//         // Initialize pose estimator
//         poseEstimator = new PhotonPoseEstimator(
//             fieldLayout,
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//             camera,
//             robotToCam
//         );
//     }

//     @Override
//     public void periodic() {
//         // Update latest result
//         latestResult = camera.getLatestResult();
//     }

//     // Get robot's field-relative pose from vision
//     public Optional<Pose2d> getRobotPose() {
//         var result = poseEstimator.update(latestResult);
//         return result.isPresent() ? Optional.of(result.get().estimatedPose.toPose2d()) : Optional.empty();
//     }

//     // Get the pose of a specific AprilTag
//     public Optional<Pose2d> getTagPose(int tagId) {
//         return fieldLayout.getTagPose(tagId).isPresent() 
//             ? Optional.of(fieldLayout.getTagPose(tagId).get().toPose2d()) 
//             : Optional.empty();
//     }

//     // Check if a specific tag is visible
//     public boolean hasTarget(int tagId) {
//         if (latestResult != null && latestResult.hasTargets()) {
//             return latestResult.getTargets().stream()
//                 .anyMatch(target -> target.getFiducialId() == tagId);
//         }
//         return false;
//     }
// }