package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultVisionDisplay extends Command{
    
    private final VisionSubsystem vision;

    private double targetYaw;
    private boolean hasTarget;

    public DefaultVisionDisplay(VisionSubsystem subsystem){
        this.vision = subsystem;
    }

    @Override
    public void execute(){
        hasTarget = vision.hasTarget();
        if (hasTarget) {
            PhotonTrackedTarget target = vision.getBestTarget();
            targetYaw = target.getYaw();
            SmartDashboard.putBoolean("left Aligned", targetYaw > -11 && targetYaw < -9);
        }
        else{
            SmartDashboard.putBoolean("left Aligned", false);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
