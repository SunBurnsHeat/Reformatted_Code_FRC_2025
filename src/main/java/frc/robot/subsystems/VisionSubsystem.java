package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraB;
    // private final PhotonCamera cameraG;
    private PhotonPipelineResult latestResultB;
    // private PhotonPipelineResult latestResultG;

    private final String cameraNameB;
    // private final String cameraNameG;

    
    // Camera position relative to robot center (in meters)
    private final Translation2d cameraOffsetB; // x (forward), y (left/right)
    private final Rotation2d cameraYawOffsetB; // Camera's yaw relative to robot heading
    private final double cameraHeightB; // z (height) in meters
    private final double cameraPitchB; // Pitch angle in degrees

    // private final Translation2d cameraOffsetG; // x (forward), y (left/right)
    // private final Rotation2d cameraYawOffsetG; // Camera's yaw relative to robot heading
    // private final double cameraHeightG; // z (height) in meters
    // private final double cameraPitchG; // Pitch angle in degrees

    private double rawYawB;
    // private double rawYawG;

    /**
     * Constructor with camera position
     * @param cameraName Name of the camera in PhotonVision
     * @param cameraXOffset Forward offset from robot center (meters)
     * @param cameraYOffset Left/right offset from robot center (meters)
     * @param cameraHeight Height from ground (meters)
     * @param cameraYawOffset Yaw offset from robot heading (degrees)
     * @param cameraPitch Pitch angle (degrees)
     */
    public VisionSubsystem() {

        cameraNameB = "GSC_BLACK";
        // cameraNameG = "GSC_GRAY";

        cameraB = new PhotonCamera(cameraNameB);
        // cameraG = new PhotonCamera(cameraNameG);

        latestResultB = new PhotonPipelineResult();
        // latestResultG = new PhotonPipelineResult();

        cameraOffsetB = new Translation2d(VisionConstants.cameraXOffsetB, VisionConstants.cameraYOffsetB);
        cameraYawOffsetB = Rotation2d.fromDegrees(VisionConstants.cameraYawOffsetB);
        cameraHeightB = VisionConstants.cameraHeightB;
        cameraPitchB = VisionConstants.cameraPitchB;

        // cameraOffsetG = new Translation2d(VisionConstants.cameraXOffsetG, VisionConstants.cameraYOffsetG);
        // cameraYawOffsetG = Rotation2d.fromDegrees(VisionConstants.cameraYawOffsetG);
        // cameraHeightG = VisionConstants.cameraHeightG;
        // cameraPitchG = VisionConstants.cameraPitchG;
    }

    @Override
    public void periodic() {
        latestResultB = cameraB.getLatestResult();
        // latestResultG = cameraG.getLatestResult();

        rawYawB = latestResultB.getBestTarget().getYaw();
        // rawYawG = latestResultG.getBestTarget().getYaw();

        SmartDashboard.putNumber("cameraB Yaw", rawYawB);
        // SmartDashboard.putNumber("camraG Yaw", rawYawG);

        SmartDashboard.putBoolean("isLeftAligned", isLeftAlignB());
    }


    public boolean hasTargetB() {
        return latestResultB.hasTargets();
    }

    // public boolean hasTargetG() {
    //     return latestResultG.hasTargets();
    // }

    /**
     * Get yaw adjusted for camera offset
     * @param robotHeading Current robot heading (Rotation2d)
     * @return Yaw in degrees relative to robot frame
     */
    public double getTargetYawAdjustedB(Rotation2d robotHeading) {
        if (hasTargetB()) {
            double rawYaw = latestResultB.getBestTarget().getYaw();
            // Adjust yaw for camera's orientation and position
            Rotation2d adjustedYawB = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffsetB).minus(robotHeading);
            return adjustedYawB.getDegrees();
        }
        return 0.0;
    }


    // public double getTargetYawAdjustedG(Rotation2d robotHeading) {
    //     if (hasTargetG()) {
    //         double rawYaw = latestResultG.getBestTarget().getYaw();
    //         // Adjust yaw for camera's orientation and position
    //         Rotation2d adjustedYawG = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffsetG).minus(robotHeading);
    //         return adjustedYawG.getDegrees();
    //     }
    //     return 0.0;
    // }

    public boolean isLeftAlignB(){
        if (hasTargetB()) {
            return (rawYawB > VisionConstants.leftAlignRangeLeftInterval) && (rawYawB < VisionConstants.leftAlignRangeRightInterval);
        }
        else{
            return false;
        }
    }

    // public boolean isLeftAlignG(){
    //     if (hasTargetG()) {
    //         return (rawYawG > VisionConstants.leftAlignRangeLeftInterval) && (rawYawG < VisionConstants.leftAlignRangeRightInterval);
    //     }
    //     else{
    //         return false;
    //     }

    // }


    public double getTargetPitchB() {
        if (hasTargetB()) {
            return latestResultB.getBestTarget().getPitch();
        }
        return 0.0;
    }

    // public double getTargetPitchG() {
    //     if (hasTargetG()) {
    //         return latestResultG.getBestTarget().getPitch();
    //     }
    //     return 0.0;
    // }

    public double getTargetAreaB() {
        if (hasTargetB()) {
            return latestResultB.getBestTarget().getArea();
        }
        return 0.0;
    }

    // public double getTargetAreaG() {
    //     if (hasTargetG()) {
    //         return latestResultG.getBestTarget().getArea();
    //     }
    //     return 0.0;
    // }
    /**
     * Estimate distance to target, accounting for camera height and pitch
     * @param targetHeight Height of target from ground (meters)
     * @return Distance in meters, -1 if no target
     */
    public double getDistanceToTargetB(double targetHeight) {
        if (hasTargetB()) {
            double pitch = getTargetPitchB();
            double totalPitch = Math.toRadians(cameraPitchB + pitch);
            return (targetHeight - cameraHeightB) / Math.tan(totalPitch);
        }
        return -1.0;
    }

    // public double getDistanceToTargetG(double targetHeight) {
    //     if (hasTargetG()) {
    //         double pitch = getTargetPitchG();
    //         double totalPitch = Math.toRadians(cameraPitchG + pitch);
    //         return (targetHeight - cameraHeightG) / Math.tan(totalPitch);
    //     }
    //     return -1.0;
    // }

    public PhotonTrackedTarget getBestTargetB() {
        if (hasTargetB()) {
            return latestResultB.getBestTarget();
        }
        return null;
    }

    // public PhotonTrackedTarget getBestTargetG() {
    //     if (hasTargetG()) {
    //         return latestResultG.getBestTarget();
    //     }
    //     return null;
    // }

    public Translation2d getCameraOffsetB() {
        return cameraOffsetB;
    }
    // public Translation2d getCameraOffsetG() {
    //     return cameraOffsetG;
    // }

    public Rotation2d getCameraYawOffsetB() {
        return cameraYawOffsetB;
    }
    // public Rotation2d getCameraYawOffsetG() {
    //     return cameraYawOffsetG;
    // }
}