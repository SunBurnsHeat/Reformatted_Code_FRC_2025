package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;

public class PoseEstimator extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonTrackedTarget latestTarget; // Store the latest detected target
    private PhotonPipelineResult latestResult; // Store the latest pipeline result

    // Constructor
    public PoseEstimator() {
        CommandScheduler.getInstance().registerSubsystem(this);
        
        camera = new PhotonCamera("GSC_BLACK");

        try {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }

        Transform3d robotToCamera = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5), // Adjust based on your camera position
            new Rotation3d(0.0, 0.0, 0.0)
        );

        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            robotToCamera
        );
        photonPoseEstimator.setReferencePose(new Pose2d());
    }

    @Override
    public void periodic() {
        // Get the latest pipeline result from the camera
        latestResult = camera.getLatestResult();
        
        // Update the pose estimator with the pipeline result
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update(latestResult);
        
        if (result.isPresent()) {
            Pose3d estimatedPose = result.get().estimatedPose;
            
            if (latestResult.hasTargets()) {
                latestTarget = latestResult.getBestTarget();
                
                Pose3d targetPose = estimatedPose.plus(latestTarget.getBestCameraToTarget());
                SmartDashboard.putNumber("Target Pose X", targetPose.getX());
                SmartDashboard.putNumber("Target Pose Y", targetPose.getY());
                SmartDashboard.putNumber("Target Pose Z", targetPose.getZ());
                SmartDashboard.putNumber("Target Rotation Yaw", targetPose.getRotation().getZ());
            } else {
                latestTarget = null;
                SmartDashboard.putString("Target Status", "No targets detected");
            }
        } else {
            latestTarget = null; // Clear target if no valid pose estimate
            SmartDashboard.putString("Pose Status", "No valid pose estimate");
        }
    }

    // --- Target Property Acquisition Methods ---

    /**
     * Checks if a target is currently detected.
     * @return true if a target is detected, false otherwise
     */
    public boolean hasTarget() {
        return latestTarget != null;
    }

    /**
     * Gets the ID of the detected AprilTag.
     * @return the AprilTag ID, or -1 if no target is detected
     */
    public int getTargetId() {
        return hasTarget() ? latestTarget.getFiducialId() : -1;
    }

    /**
     * Gets the field-relative pose of the detected target.
     * @return the target's Pose3d, or null if no target is detected
     */
    public Pose3d getTargetPose() {
        if (!hasTarget()) return null;
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update(latestResult);
        if (result.isPresent()) {
            Pose3d robotPose = result.get().estimatedPose;
            return robotPose.plus(latestTarget.getBestCameraToTarget());
        }
        return null;
    }

    /**
     * Calculates the distance from the camera to the target.
     * @return distance in meters, or -1 if no target is detected
     */
    public double getTargetDistance() {
        if (!hasTarget()) return -1.0;
        Transform3d cameraToTarget = latestTarget.getBestCameraToTarget();
        return cameraToTarget.getTranslation().getNorm();
    }

    /**
     * Gets the yaw angle of the target relative to the camera.
     * @return yaw in degrees, or 0 if no target is detected
     */
    public double getTargetYaw() {
        return hasTarget() ? latestTarget.getYaw() : 0.0;
    }

    /**
     * Gets the pitch angle of the target relative to the camera.
     * @return pitch in degrees, or 0 if no target is detected
     */
    public double getTargetPitch() {
        return hasTarget() ? latestTarget.getPitch() : 0.0;
    }

    /**
     * Gets the area of the target in the camera's field of view.
     * @return area as a percentage (0.0 to 100.0), or 0 if no target is detected
     */
    public double getTargetArea() {
        return hasTarget() ? latestTarget.getArea() : 0.0;
    }

    /**
     * Gets the latest estimated robot pose.
     * @return Optional containing the robot's Pose3d, or empty if no valid estimate
     */
    public Optional<Pose3d> getEstimatedPose() {
        return photonPoseEstimator.update(latestResult).map(estimated -> estimated.estimatedPose);
    }
}