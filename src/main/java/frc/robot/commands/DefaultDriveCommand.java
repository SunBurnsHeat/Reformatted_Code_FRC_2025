package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// public class DefaultDriveCommand extends Command{

//     // drive subsystem object
//     private final DriveSubsystem driveSubsystem;
//     // private final VisionSubsystem visionSubsystem;
//     // the controller's object
//     private XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

//     // // Vision alignment constants
//     // private static final double VISION_TURN_KP = 0.025;
//     // private static final int TARGET_FIDUCIAL_ID = 7;
//     // private static final double AIMING_DEADBAND = 2.0;
    
//     // the class's constructor
//     public DefaultDriveCommand(DriveSubsystem subsystem/* *, VisionSubsystem vision*/){
//         this.driveSubsystem = subsystem; // assigns given subsystem to drive
//         // this.visionSubsystem = vision;
//         addRequirements(subsystem); // enforces argument to prevent conflict
//     }

//     @Override
//     // called repeatedly when cammand is called
//     // contains the logic for robot's movement or this command 
//     public void execute(){
//         int fineTurn = 0; // robot's turning sensitivity when received controller input
//         if( driverController.getXButton() ){
//             fineTurn += 1; // if button X is pressed, the slow turning constant is set to 1 or left_turn
//         }
//         if( driverController.getBButton() ){
//             fineTurn -= 1;  // if button B is pressed, the slow turning constant is set to -1 or right_turn
//         }

//         double multiplier = 0.4; // speed adjustment constant for regular control
//         double povMultiplier = 0.5; // speed adjustment constant for POV (D-Pad) control
//         // when both triggers are pressed, ....
//         if( (driverController. getLeftTriggerAxis() > 0.75 ) && (driverController.getRightTriggerAxis() > 0.75) ) {
//             multiplier = 1; // turbo mode (100% speed) for regular control
//             povMultiplier = 1.5; // turbo mode (150% speed) for POV control
//         }
//         // when only one trigger is pressed, ....
//         else if( (driverController.getLeftTriggerAxis() > 0.75) || (driverController.getRightTriggerAxis() > 0.75) ) {
//         multiplier = 0.65; // moderate mode (65% speed) for regular control
//         povMultiplier = 1;  // normal mode (100% speed) for POV control
//         }

//         /*------------------------------PhotonVision Block------------------------------------ */

//         // // Manual swerve inputs for photon steering
//         // double forward = -multiplier * MathUtil.applyDeadband(driverController.getLeftY(), 0.015) * DriveConstants.kMaxSpeedMetersPerSec;
//         // double strafe = -multiplier * MathUtil.applyDeadband(driverController.getLeftX(), 0.015) * DriveConstants.kMaxSpeedMetersPerSec;
//         // double rotation = -multiplier * 0.85 * MathUtil.applyDeadband(driverController.getRightX(), 0.01) * DriveConstants.kMaxAngSpeedRadiansPerSec;

//         // boolean targetVisible = false;
//         // double targetYaw = 0.0;
//         // if (driverController.getAButton() && visionSubsystem.hasTarget()) {
//         //     PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//         //     if (target.getFiducialId() > 0) {
//         //         Rotation2d robotHeading = driveSubsystem.getHeadingRotation2d();
//         //         targetYaw = visionSubsystem.getTargetYawAdjusted(robotHeading);
//         //         targetVisible = true;

//         //         if (Math.abs(targetYaw) > AIMING_DEADBAND) {
//         //             rotation = - targetYaw*VISION_TURN_KP*DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //         }
//         //         else{
//         //             rotation = 0.0;
//         //         }

//         //         Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//         //         double distance = visionSubsystem.getDistanceToTarget(1.2);
//         //         if (distance > 0) {
                    
//         //             double yawRad = Math.toRadians(targetYaw);
//         //             forward -= cameraOffset.getX()*Math.cos(yawRad)*multiplier;
//         //             strafe -= cameraOffset.getY()*Math.sin(yawRad)*multiplier;
//         //         }
//         //     }
//         // }

//         /*------------------------------PhotonVision Block------------------------------------ */


//         // // when Button A is pressed, ....
//         //  if(driverController.getAButton()) {
//         //     // turns on the lime_light
//         //     LimelightHelpers.setLEDMode_ForceOn("limelight");
//         //     // retrieves lime_light data
//         //     LimelightResults results = LimelightHelpers.getLatestResults("limelight");
//         //     // if there is any visual input for value (binary) & the value translates to ID 7 or ID 4 target, ....
//         //     if(results.targetingResults.targets_Fiducials.length > 0 && 
//         //     (results.targetingResults.targets_Fiducials[0].fiducialID == 7 || 
//         //         results.targetingResults.targets_Fiducials[0].fiducialID == 4)) {
                
//         //         // calculate target angle 
//         //         double angle = Math.atan(results.targetingResults.targets_Fiducials[0]
//         //             .getTargetPose_CameraSpace().getX() /
//         //             results.targetingResults.targets_Fiducials[0]
//         //             .getTargetPose_CameraSpace().getZ());
                
//         //         // the angle is displayed in the dashboard UI
//         //         SmartDashboard.putNumber("angle", angle);
                
//         //         // if the target angle is greater than the given prox value, ....
//         //         if(Math.abs(angle) > SmartDashboard.getNumber("Aiming deadband", 0.05)) {
//         //             // drive this assistive way 
//         //             driveSubsystem.drive(-multiplier*MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
//         //                                 -multiplier*MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
//         //                                 Math.copySign(Math.abs(angle / 
//         //                                 SmartDashboard.getNumber("Aiming kp", 5.0)) + 
//         //                                 SmartDashboard.getNumber("Aiming minsteer", 0.035), -angle), 
//         //                                 true, true);
//         //         } 
//         //         // otherwise (target is not near), ....
//         //         else {
//         //             // drive normally without rotating
//         //             driveSubsystem.drive(-multiplier*MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
//         //                                 -multiplier*MathUtil.applyDeadband(driverController.getLeftX(), 0.015), 
//         //                                 0, true, true);
//         //         }
//         //         return;
//         //     }
//         // } 
//         // // otherwise (Button A not pressed), ....
//         // else {
//         //     // turn the lime_light led off 
//         //     LimelightHelpers.setLEDMode_ForceOff("limelight");
//         // }

//         /*------------------------------PhotonVision Block------------------------------------ */

//         // // POV (D-pad) control overrides joystick if active
//         // if (driverController.getPOV() != -1) {
//         //     switch (driverController.getPOV()) {
//         //         case 0:   // Up
//         //             forward = povMultiplier * 0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             strafe = 0;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 45:  // Up-right
//         //             forward = povMultiplier * 0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             strafe = povMultiplier * -0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 90:  // Right
//         //             forward = 0;
//         //             strafe = povMultiplier * -0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 135: // Down-right
//         //             forward = povMultiplier * -0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             strafe = povMultiplier * -0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 180: // Down
//         //             forward = povMultiplier * -0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             strafe = 0;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 225: // Down-left
//         //             forward = povMultiplier * -0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             strafe = povMultiplier * 0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 270: // Left
//         //             forward = 0;
//         //             strafe = povMultiplier * 0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //         case 315: // Up-left
//         //             forward = povMultiplier * 0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             strafe = povMultiplier * 0.25 * DriveConstants.kMaxSpeedMetersPerSec;
//         //             rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         //             break;
//         //     }
//         // } else if (fineTurn != 0 && !targetVisible) {
//         //     // Apply fine turn if no POV or vision alignment is active
//         //     rotation = povMultiplier * fineTurn * DriveConstants.kMaxAngSpeedRadiansPerSec;
//         // }

//         // // Command the swerve drivetrain
//         // driveSubsystem.drive(forward, strafe, rotation, true, true);

//         // // Debug info
//         // SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
//         // SmartDashboard.putNumber("Target Yaw", targetYaw);

//         /*------------------------------PhotonVision Block------------------------------------ */

//         // if no D-pad direction pressed, ....
//         if(driverController.getPOV() == -1){
//             // if there is not turning input, ....
//             if(fineTurn == 0){
//                 // drive without the fine_turn constant
//                driveSubsystem.drive(
//                     -multiplier*MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
//                     -multiplier*MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
//                     -multiplier*0.85*MathUtil.applyDeadband(driverController.getRightX(), 0.01),
//                     true, true); 
//             }
//             // otherwise, ....
//             else {
//                 // drive with the constant
//                 driveSubsystem.drive(
//                     -multiplier*MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
//                     -multiplier*MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
//                     povMultiplier*fineTurn,
//                     true, true);
//             }
//         }
//         // otherwise(if D-pad direction pressed), ....
//         else {
//             // when the D-pad's angle is ....
//             switch(driverController.getPOV()){
//                 // drive towards those direction
//                 case(0):
//                     driveSubsystem.drive(povMultiplier*0.25, 0, povMultiplier*fineTurn, true, true); 
//                 case(45):
//                     driveSubsystem.drive(povMultiplier*0.25, povMultiplier*-0.25, povMultiplier*fineTurn, true, true);
//                 case(90):
//                     driveSubsystem.drive(0, povMultiplier*-0.25, povMultiplier*fineTurn, true, true);
//                 case(135):
//                     driveSubsystem.drive(povMultiplier*-0.25, povMultiplier*-0.25, povMultiplier*fineTurn, true, true);
//                 case(180):
//                     driveSubsystem.drive(povMultiplier*-0.25, 0, povMultiplier*fineTurn, true, true);
//                 case(225):
//                     driveSubsystem.drive(povMultiplier*-0.25, povMultiplier*0.25, povMultiplier*fineTurn, true, true);
//                 case(270):
//                     driveSubsystem.drive(0, povMultiplier*0.25, povMultiplier*fineTurn, true, true);
//                 case(315):
//                     driveSubsystem.drive(povMultiplier*0.25, povMultiplier*0.25, povMultiplier*fineTurn, true, true);
//             }
//         }
//     }

//     @Override
//     // this class's command is not ended until interrupted
//     public boolean isFinished() {
//         return false;
//     }
// }

    /* -------------------------------- Added alignment command ------------------------------------- */

    public class DefaultDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem; // Add VisionSubsystem
    private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
    private AlignToTagCommand alignCommand; // Instance to manage alignment

    public DefaultDriveCommand(DriveSubsystem subsystem, VisionSubsystem vision) {
        this.driveSubsystem = subsystem;
        this.visionSubsystem = vision;
        addRequirements(subsystem, vision); // VisionSubsystem isn’t strictly required here
    }

    @Override
    public void initialize() {
        // Ensure no alignment command is running at start
        if (alignCommand != null && alignCommand.isScheduled()) {
            alignCommand.cancel();
        }
        alignCommand = null;
    }

    @Override
    public void execute() {
        int fineTurn = 0;
        if (driverController.getXButton()) {
            fineTurn += 1; // Left turn
        }
        if (driverController.getBButton()) {
            fineTurn -= 1; // Right turn
        }

        double multiplier = 0.4; // Default speed
        double povMultiplier = 0.5; // Default POV speed
        if (driverController.getLeftTriggerAxis() > 0.75 && driverController.getRightTriggerAxis() > 0.75) {
            multiplier = 1; // Turbo mode
            povMultiplier = 1.5;
        } else if (driverController.getLeftTriggerAxis() > 0.75 || driverController.getRightTriggerAxis() > 0.75) {
            multiplier = 0.65; // Moderate mode
            povMultiplier = 1;
        }

        // Check for alignment trigger
        if (driverController.getAButton()) {
            // If alignment isn’t already running, start it
            if (alignCommand == null || !alignCommand.isScheduled()) {
                alignCommand = new AlignToTagCommand(driveSubsystem, visionSubsystem);
                alignCommand.schedule();
            }
            // Let AlignToTagCommand handle driving while it’s active
            return; // Skip manual driving logic
        } else {
            // If A button is released and alignment is running, cancel it
            if (alignCommand != null && alignCommand.isScheduled()) {
                alignCommand.cancel();
                alignCommand = null;
            }
        }

        // Manual driving logic (runs only if alignment isn’t active)
        if (driverController.getPOV() == -1) {
            if (fineTurn == 0) {
                driveSubsystem.drive(
                    -multiplier * MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
                    -multiplier * MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
                    -multiplier * 0.85 * MathUtil.applyDeadband(driverController.getRightX(), 0.01),
                    true, true
                );
            } else {
                driveSubsystem.drive(
                    -multiplier * MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
                    -multiplier * MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
                    povMultiplier * fineTurn,
                    true, true
                );
            }
        } else {
            switch (driverController.getPOV()) {
                case 0:
                    driveSubsystem.drive(povMultiplier * 0.25, 0, povMultiplier * fineTurn, true, true);
                    break;
                case 45:
                    driveSubsystem.drive(povMultiplier * 0.25, povMultiplier * -0.25, povMultiplier * fineTurn, true, true);
                    break;
                case 90:
                    driveSubsystem.drive(0, povMultiplier * -0.25, povMultiplier * fineTurn, true, true);
                    break;
                case 135:
                    driveSubsystem.drive(povMultiplier * -0.25, povMultiplier * -0.25, povMultiplier * fineTurn, true, true);
                    break;
                case 180:
                    driveSubsystem.drive(povMultiplier * -0.25, 0, povMultiplier * fineTurn, true, true);
                    break;
                case 225:
                    driveSubsystem.drive(povMultiplier * -0.25, povMultiplier * 0.25, povMultiplier * fineTurn, true, true);
                    break;
                case 270:
                    driveSubsystem.drive(0, povMultiplier * 0.25, povMultiplier * fineTurn, true, true);
                    break;
                case 315:
                    driveSubsystem.drive(povMultiplier * 0.25, povMultiplier * 0.25, povMultiplier * fineTurn, true, true);
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up alignment command if interrupted
        if (alignCommand != null && alignCommand.isScheduled()) {
            alignCommand.cancel();
        }
        driveSubsystem.drive(0, 0, 0, false, false); // Stop the robot
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
