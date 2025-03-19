package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimator; // Your pose estimator
import frc.robot.subsystems.DriveSubsystem;


// NOT USED FOR SWERVE ALIGNMENT 
public class SwerveAligningCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final PoseEstimator poseEstimator;
    private final PIDController yawController;
    private final PIDController distanceController; // Optional for distance alignment
    private final double desiredDistance; // Target distance in meters, or 0 if not used

    // Constructor for yaw alignment only
    public SwerveAligningCommand(DriveSubsystem driveSubsystem, PoseEstimator poseEstimator) {
        this.driveSubsystem = driveSubsystem;
        this.poseEstimator = poseEstimator;
        this.yawController = new PIDController(0.1, 0.0, 0.01); // Tune these gains
        this.distanceController = null;
        this.desiredDistance = 0.0;

        yawController.setTolerance(1.0); // Align within 1 degree
        addRequirements(driveSubsystem, poseEstimator);
    }

    // Constructor for yaw and distance alignment
    public SwerveAligningCommand(DriveSubsystem driveSubsystem, PoseEstimator poseEstimator, double desiredDistance) {
        this.driveSubsystem = driveSubsystem;
        this.poseEstimator = poseEstimator;
        this.yawController = new PIDController(0.1, 0.0, 0.01); // Tune yaw gains
        this.distanceController = new PIDController(0.05, 0.0, 0.005); // Tune distance gains
        this.desiredDistance = desiredDistance;

        yawController.setTolerance(1.0); // Within 1 degree
        distanceController.setTolerance(0.1); // Within 0.1 meters
        addRequirements(driveSubsystem, poseEstimator);
    }

    @Override
    public void initialize() {
        yawController.reset();
        if (distanceController != null) {
            distanceController.reset();
        }
    }

    @Override
    public void execute() {
        if (!poseEstimator.hasTarget()) {
            // No target detected, stop the robot
            driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
            return;
        }

        // Get target properties from PoseEstimator
        double targetYaw = poseEstimator.getTargetYaw(); // Degrees
        double targetDistance = poseEstimator.getTargetDistance(); // Meters

        // Calculate rotational speed (convert yaw to radians for ChassisSpeeds)
        double rotationSpeed = yawController.calculate(targetYaw, 0.0); // Setpoint is 0 (aligned)

        // Optional: Calculate forward speed for distance adjustment
        double forwardSpeed = 0.0;
        if (distanceController != null) {
            forwardSpeed = distanceController.calculate(targetDistance, desiredDistance);
        }

        // Create robot-relative chassis speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            forwardSpeed, // x-translation (forward/backward)
            0.0,          // y-translation (side-to-side, not used)
            Math.toRadians(rotationSpeed) // Rotation speed in radians/sec
        );

        // Apply speeds to the drive subsystem
        driveSubsystem.driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        boolean yawAligned = yawController.atSetpoint();
        boolean distanceAligned = (distanceController == null) || distanceController.atSetpoint();
        return yawAligned && distanceAligned && poseEstimator.hasTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}