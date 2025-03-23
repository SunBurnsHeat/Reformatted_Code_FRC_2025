package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScorerSubsystem;

public class PulseScorerCommand extends Command {
    private ScorerSubsystem scorer;
    private double pulseDutyCycle;
    private double pulseDuration; // in seconds
    private double restDuration; // in seconds
    private int pulseCount; // number of pulses

    private double lastLeftSpeed = Double.NaN;
    private double lastRightSpeed = Double.NaN;
    
    private final Timer timer;
    private int currentPulse;
    private boolean isPulsing;
    
        public PulseScorerCommand(ScorerSubsystem scorerSubsystem) {
        this.scorer = scorerSubsystem;
        pulseDutyCycle = .05;
        pulseDuration = .3;
        restDuration = .2;
        pulseCount = 8;
        
        timer = new Timer();
        currentPulse = 0;
        isPulsing = true;
        
        addRequirements(scorerSubsystem);
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        currentPulse = 0;
        isPulsing = true;
        scorer.setScorerMaxLeft(pulseDutyCycle);
        scorer.setScorerMaxRight(pulseDutyCycle);
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();
        double leftSpeed, rightSpeed;

        if (isPulsing && elapsedTime >= pulseDuration) {
            leftSpeed = 0.0; rightSpeed = 0.0;
            timer.reset();
            isPulsing = false;
        } else if (!isPulsing && elapsedTime >= restDuration) {
            currentPulse++;
            if (currentPulse < pulseCount) {
                leftSpeed = pulseDutyCycle; rightSpeed = pulseDutyCycle;
                timer.reset();
                isPulsing = true;
            } else {
                leftSpeed = 0.0; rightSpeed = 0.0;
            }
        } else {
            return; // No change, skip motor updates
        }

        if (leftSpeed != lastLeftSpeed) {
            scorer.setScorerMaxLeft(leftSpeed);
            lastLeftSpeed = leftSpeed;
        }
        if (rightSpeed != lastRightSpeed) {
            scorer.setScorerMaxRight(rightSpeed);
            lastRightSpeed = rightSpeed;
        }
    }
    
    // @Override
    // public void execute() {
    //     double elapsedTime = timer.get();
        
    //     if (isPulsing && elapsedTime >= pulseDuration) {
    //         scorer.stop();
    //         timer.reset();
    //         isPulsing = false;
    //     } 
    //     else if (!isPulsing && elapsedTime >= restDuration) {
    //         currentPulse++;
    //         if (currentPulse < pulseCount) {
    //             scorer.setScorerMaxLeft(pulseDutyCycle);
    //             scorer.setScorerMaxRight(pulseDutyCycle);
    //             timer.reset();
    //             isPulsing = true;
    //         }
    //     }
    // }
    
    @Override
    public void end(boolean interrupted) {
        scorer.stop();
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        return currentPulse >= pulseCount && !isPulsing;
    }
}