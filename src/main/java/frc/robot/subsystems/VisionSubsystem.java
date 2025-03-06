package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private final String cameraName;
    
    // Camera position relative to robot center (in meters)
    private final Translation2d cameraOffset; // x (forward), y (left/right)
    private final Rotation2d cameraYawOffset; // Camera's yaw relative to robot heading
    private final double cameraHeight; // z (height) in meters
    private final double cameraPitch; // Pitch angle in degrees

    /**
     * Constructor with camera position
     * @param cameraName Name of the camera in PhotonVision
     * @param cameraXOffset Forward offset from robot center (meters)
     * @param cameraYOffset Left/right offset from robot center (meters)
     * @param cameraHeight Height from ground (meters)
     * @param cameraYawOffset Yaw offset from robot heading (degrees)
     * @param cameraPitch Pitch angle (degrees)
     */
    public VisionSubsystem(String cameraName, double cameraXOffset, double cameraYOffset, double cameraHeight,
                          double cameraYawOffset, double cameraPitch) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        latestResult = new PhotonPipelineResult();
        this.cameraOffset = new Translation2d(cameraXOffset, cameraYOffset);
        this.cameraYawOffset = Rotation2d.fromDegrees(cameraYawOffset);
        this.cameraHeight = cameraHeight;
        this.cameraPitch = cameraPitch;
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
    }

    public String getCameraName() {
        return cameraName;
    }

    public boolean hasTarget() {
        return latestResult.hasTargets();
    }

    /**
     * Get yaw adjusted for camera offset
     * @param robotHeading Current robot heading (Rotation2d)
     * @return Yaw in degrees relative to robot frame
     */
    public double getTargetYawAdjusted(Rotation2d robotHeading) {
        if (hasTarget()) {
            double rawYaw = latestResult.getBestTarget().getYaw();
            // Adjust yaw for camera's orientation and position
            Rotation2d adjustedYaw = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
            return adjustedYaw.getDegrees();
        }
        return 0.0;
    }

    public double getTargetPitch() {
        if (hasTarget()) {
            return latestResult.getBestTarget().getPitch();
        }
        return 0.0;
    }

    public double getTargetArea() {
        if (hasTarget()) {
            return latestResult.getBestTarget().getArea();
        }
        return 0.0;
    }

    /**
     * Estimate distance to target, accounting for camera height and pitch
     * @param targetHeight Height of target from ground (meters)
     * @return Distance in meters, -1 if no target
     */
    public double getDistanceToTarget(double targetHeight) {
        if (hasTarget()) {
            double pitch = getTargetPitch();
            double totalPitch = Math.toRadians(cameraPitch + pitch);
            return (targetHeight - cameraHeight) / Math.tan(totalPitch);
        }
        return -1.0;
    }

    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget()) {
            return latestResult.getBestTarget();
        }
        return null;
    }

    public Translation2d getCameraOffset() {
        return cameraOffset;
    }

    public Rotation2d getCameraYawOffset() {
        return cameraYawOffset;
    }
}