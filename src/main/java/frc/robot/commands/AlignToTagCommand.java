package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private SwerveControllerCommand swerveCommand;
    private int currentTagId = -1; // Track the tag we’re aligning to

    // Camera-to-robot transform
    private final Transform3d robotToCam = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5), // x, y, z in meters
        new Rotation3d(0, Math.toRadians(-30), 0) // roll, pitch, yaw
    );

    // PID controllers for trajectory following
    private final PIDController xController = new PIDController(5.0, 0, 0); // Tune these
    private final PIDController yController = new PIDController(5.0, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        5.0, 0, 0,
        new TrapezoidProfile.Constraints(
            DriveConstants.kMaxAngSpeedRadiansPerSec,
            DriveConstants.kMaxAngSpeedRadiansPerSec * 2) // Max vel, accel for rotation
    );

    public AlignToTagCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;
        addRequirements(drive, vision);
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // Handle angle wrapping
    }

    @Override
    public void initialize() {
        // Get current pose from DriveSubsystem's estimator
        Pose2d currentPose = driveSubsystem.getEstPose2d();

        // Get the best tag pose from VisionSubsystem
        var tagPoseOpt = visionSubsystem.getBestTagPose();
        if (!tagPoseOpt.isPresent()) {
            swerveCommand = null;
            return;
        }

        Pose2d tagPose = tagPoseOpt.get();
        currentTagId = visionSubsystem.getBestTagId().get(); // Store the tag ID for vision fusion

        // Define desired pose 
        Pose2d desiredPose = new Pose2d(
            tagPose.getX() - Units.inchesToMeters(10), // need to change based on the desired distance 
            tagPose.getY(),
            tagPose.getRotation()
        );

        // Generate trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            DriveConstants.kMaxSpeedMetersPerSec,
            DriveConstants.kMaxSpeedMetersPerSec / 3 // Max accel
        ).setKinematics(DriveConstants.kDriveKinematics)
         .addConstraint(new SwerveDriveKinematicsConstraint(DriveConstants.kDriveKinematics, DriveConstants.kMaxSpeedMetersPerSec));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            currentPose,
            List.of(), // No waypoints (direct path)
            desiredPose,
            config
        );

        // Create swerve command
        swerveCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getEstPose2d, // Pose from estimator
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            driveSubsystem::setModuleStates, // Module state setter
            driveSubsystem
        );

        swerveCommand.initialize();
    }

    @Override
    public void execute() {
    if (swerveCommand != null) {
            swerveCommand.execute();

            var visionResultOpt = visionSubsystem.getLatestVisionResult();
            if (visionResultOpt.isPresent()) {
                PhotonPipelineResult result = visionResultOpt.get();
                var target = result.getBestTarget();
                if (target.getFiducialId() == currentTagId) {
                    Pose2d tagPose = visionSubsystem.getBestTagPose().get();
                    Transform3d camToTag = target.getBestCameraToTarget();

                    // Convert 3D transforms to 2D
                    Transform2d camToTag2d = toTransform2d(camToTag.inverse());
                    Transform2d robotToCam2d = toTransform2d(robotToCam.inverse());

                    // Apply transforms in 2D
                    Pose2d camPose = tagPose.transformBy(camToTag2d);
                    Pose2d robotPose = camPose.transformBy(robotToCam2d);

                    driveSubsystem.addVisionMeasurement(robotPose, result.getTimestampSeconds());
                }
            }
        }    
    }

    @Override
    public void end(boolean interrupted) {
        if (swerveCommand != null) {
            swerveCommand.end(interrupted);
        }
        driveSubsystem.drive(0, 0, 0, false, false); // Stop the robot
        currentTagId = -1; // Reset tag ID
    }

    @Override
    public boolean isFinished() {
        return swerveCommand != null && swerveCommand.isFinished();
    }

    // Helper method to convert Transform3d to Transform2d
    private Transform2d toTransform2d(Transform3d transform3d) {
        return new Transform2d(
            new Translation2d(transform3d.getX(), transform3d.getY()), // x, y only
            Rotation2d.fromRadians(transform3d.getRotation().getZ())   // yaw only
        );
    }
}