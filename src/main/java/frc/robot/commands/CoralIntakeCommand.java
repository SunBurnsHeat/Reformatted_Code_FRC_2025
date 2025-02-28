package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScorerSubsystem;

public class CoralIntakeCommand extends Command {
    private ScorerSubsystem scorer;

    public CoralIntakeCommand(ScorerSubsystem subsystem){
        this.scorer = subsystem;
        addRequirements(subsystem);
    }

    /**
     * Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        scorer.setScorerMaxLeft(.3);
        scorer.setScorerMaxRight(.3);
    }

    /**
     * Called repeatedly while the command is scheduled.
     * This is where the command's main logic is executed.
     */
    @Override
    public void execute() {
        if ((scorer.endProx) && !(scorer.initProx)) {
            scorer.setScorerMaxLeft(0);
            scorer.setScorerMaxRight(0);    
        }
        else if ((!scorer.endProx) && (scorer.initProx)) {
            scorer.setScorerMaxLeft(0.1);
            scorer.setScorerMaxRight(0.1);    
        }
        else{
            scorer.setScorerMaxLeft(0.3);
            scorer.setScorerMaxRight(0.3);    
        }
    }

    /**
     * Called once when the command ends or is interrupted.
     * Use this to stop motors or reset states.
     * @param interrupted True if the command was interrupted, false if it ended normally.
     */
    @Override
    public void end(boolean interrupted) {
        scorer.setScorerMaxLeft(0);
        scorer.setScorerMaxRight(0);
    }

    /**
     * Determines when the command should end.
     * This should return true when the command's task is complete.
     */
    @Override
    public boolean isFinished() {
        return ((scorer.endProx) && (!scorer.initProx)); // Change this condition based on when you want the command to end
    }
}
