package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Pose2d targetOffset; 

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private final double positionTolerance = 0.05; // Meters (5 cm)
    private final double rotationTolerance = 1.0; // Degrees

    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    }

    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose2d targetOffset) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffset = targetOffset;

        this.xController = new PIDController(1.0, 0.0, 0.1); 
        this.yController = new PIDController(1.0, 0.0, 0.1);
        this.thetaController = new PIDController(2.0, 0.0, 0.2);

        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);
        thetaController.setTolerance(rotationTolerance);

        thetaController.enableContinuousInput(-180.0, 180.0);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            PhotonTrackedTarget target = visionSubsystem.getBestTarget();
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            Pose2d targetPoseRelative = new Pose2d(
                cameraToTarget.getX(),
                cameraToTarget.getY(),
                Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
            );

            Translation2d cameraOffset = visionSubsystem.getCameraOffset();
            Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
                new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
            );

            Pose2d desiredPose = targetPoseRobotRelative.transformBy(
                new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
            );

            Pose2d currentPose = driveSubsystem.getEstPose2d();

            double xSpeed = xController.calculate(currentPose.getX(), desiredPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), desiredPose.getY());
            double rotationSpeed = thetaController.calculate(
                currentPose.getRotation().getDegrees(),
                desiredPose.getRotation().getDegrees()
            );

            xSpeed = Math.max(-0.5, Math.min(0.5, xSpeed)); // Meters/sec
            ySpeed = Math.max(-0.5, Math.min(0.5, ySpeed)); // Meters/sec
            rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed)); // Radians/sec

            driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, false, false);

            Pose2d visionPose = targetPoseRobotRelative;
            double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            driveSubsystem.addVisionMeasurement(visionPose, timestamp);
        } else {
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTarget()) {
            return true; 
        }

        Pose2d currentPose = driveSubsystem.getEstPose2d();
        PhotonTrackedTarget target = visionSubsystem.getBestTarget();
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose2d targetPoseRelative = new Pose2d(
            cameraToTarget.getX(),
            cameraToTarget.getY(),
            Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
        );
        Translation2d cameraOffset = visionSubsystem.getCameraOffset();
        Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
            new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
        );
        Pose2d desiredPose = targetPoseRobotRelative.transformBy(
            new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
        );

        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
}



/*------------------------------------Version 2----------------------------------------*/

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;
//     private final Pose2d targetOffset; 
//     private final double positionTolerance = 0.01;
//     private final double rotationTolerance = 2.0; 
//     private final double translationSpeedFactor = 0.5; 
//     private final double rotationSpeedFactor = 0.02; 

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffset = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)); 
//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose2d targetOffset) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffset = targetOffset; 
//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {
//             PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//             Transform3d cameraToTarget = target.getBestCameraToTarget();
            
//             Pose2d targetPoseRelative = new Pose2d(
//                 cameraToTarget.getX(),
//                 cameraToTarget.getY(),
//                 Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//             );

//             Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//             Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//                 new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//             );

//             Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//                 new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//             );

//             Pose2d currentPose = driveSubsystem.getEstPose2d();

//             double xError = desiredPose.getX() - currentPose.getX();
//             double yError = desiredPose.getY() - currentPose.getY();
//             double thetaError = desiredPose.getRotation().minus(currentPose.getRotation()).getDegrees();

//             double xSpeed = xError * translationSpeedFactor;
//             double ySpeed = yError * translationSpeedFactor;
//             double rotationSpeed = thetaError * rotationSpeedFactor;

//             xSpeed = Math.max(-0.5, Math.min(0.5, xSpeed)); // Meters/sec
//             ySpeed = Math.max(-0.5, Math.min(0.5, ySpeed)); // Meters/sec
//             rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed)); // Radians/sec

//             driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, false, false);

//             Pose2d visionPose = targetPoseRobotRelative; // Could refine this further
//             double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
//             driveSubsystem.addVisionMeasurement(visionPose, timestamp);
//         } else {
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget()) {
//             return true;
//         }

//         Pose2d currentPose = driveSubsystem.getEstPose2d();
//         PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//         Transform3d cameraToTarget = target.getBestCameraToTarget();
//         Pose2d targetPoseRelative = new Pose2d(
//             cameraToTarget.getX(),
//             cameraToTarget.getY(),
//             Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//         );
//         Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//         Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//             new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//         );
//         Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//             new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//         );

//         double xError = Math.abs(desiredPose.getX() - currentPose.getX());
//         double yError = Math.abs(desiredPose.getY() - currentPose.getY());
//         double thetaError = Math.abs(desiredPose.getRotation().minus(currentPose.getRotation()).getDegrees());

//         return xError < positionTolerance && yError < positionTolerance && thetaError < rotationTolerance;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }
// }