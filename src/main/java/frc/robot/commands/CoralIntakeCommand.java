package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ScorerSubsystem;

public class CoralIntakeCommand extends Command {
    private ScorerSubsystem scorer;
    private int counter;

    public CoralIntakeCommand(ScorerSubsystem subsystem){
        this.scorer = subsystem;
        addRequirements(subsystem);
    }

    /**
     * Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // scorer.setScorerMaxLeft(.28);
        // scorer.setScorerMaxRight(.28);
        counter = 0;
    }

    /**
     * Called repeatedly while the command is scheduled.
     * This is where the command's main logic is executed.
     */
    @Override
    public void execute() {
        SmartDashboard.putNumber("State Counter", counter);
        switch (counter) {
            case 0:
            scorer.setScorerMaxLeft(.28);
            scorer.setScorerMaxRight(.28);
                if (scorer.initProx) {
                    LedSubsystem.setYellowMsg();
                    counter=1;
                }
                break;
            case 1:
                scorer.setScorerMaxLeft(.2);
                scorer.setScorerMaxRight(.2);
                if ((scorer.endProx) && !(scorer.initProx)) {
                    counter = 2;
                }
                break;
            case 2: 
                scorer.setScorerMaxLeft(-0.07);
                scorer.setScorerMaxRight(-0.07);
                if ((scorer.endProx) && (scorer.initProx)) {
                    counter = 3;
                }
                break;
            case 3:
                scorer.setScorerMaxLeft(0.1);
                scorer.setScorerMaxRight(0.1);
                if ((scorer.endProx) && !(scorer.initProx)) {
                    LedSubsystem.setGreenMsg();
                    counter = 4;
                }
                break;
            case 4:
                scorer.setScorerMaxLeft(0.0);
                scorer.setScorerMaxRight(0.0);
                break;

        }
        // else if ((!scorer.endProx) && (scorer.initProx)) {
        //     scorer.setScorerMaxLeft(0.12);
        //     scorer.setScorerMaxRight(0.12);    
        // }
        // else if ((scorer.endProx) && (scorer.initProx)) {
        //     scorer.setScorerMaxLeft(0);
        //     scorer.setScorerMaxRight(0);  
        // }
        // else {
        //     scorer.setScorerMaxLeft(0.285);
        //     scorer.setScorerMaxRight(0.285);
        // }

        // if (!scorer.initProx) {
        //     scorer.setScorerMaxLeft(0);
        //     scorer.setScorerMaxRight(0);
        // }
        // else{
        //     scorer.setScorerMaxLeft(0.3);
        //     scorer.setScorerMaxRight(0.3);
        // }
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
        // return ((scorer.endProx) && (!scorer.initProx)); // Change this condition based on when you want the command to end
        // return ((scorer.endProx) && !(scorer.initProx));
        return counter == 4;
    }
}
