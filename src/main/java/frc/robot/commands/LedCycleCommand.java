package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ScorerSubsystem;

public class LedCycleCommand extends Command {
    private final LedSubsystem ledSubsystem;
    private final ScorerSubsystem scorer;

    public LedCycleCommand(LedSubsystem ledSubsystem, ScorerSubsystem scorer) {
        this.ledSubsystem = ledSubsystem;
        this.scorer = scorer;
        addRequirements(ledSubsystem); // This command requires the LedSubsystem
    }

    @Override
    public void initialize() {
        // Optional: Reset LEDs to a known state
        ledSubsystem.setBlank();
    }

    @Override
    public void execute() {
        // Same logic as the original periodic()
        if (DriverStation.isDisabled()) {
            ledSubsystem.setScroll();
        } else if (DriverStation.isAutonomous()) {
            ledSubsystem.setRainbow();
        } else if (DriverStation.isTeleop()) {
            double timeRemaining = DriverStation.getMatchTime();
            if (timeRemaining > 0 && timeRemaining < 20) {
                ledSubsystem.setBreathing();
            } else if (scorer.hasCoral()) {
                ledSubsystem.setGreen();
            } else if (scorer.ongoingCoral()) {
                ledSubsystem.setYellow();
            } else {
                ledSubsystem.setAllianceSolid();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs indefinitely as a default command
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: Set LEDs to a safe state when interrupted
        ledSubsystem.setBlank();
    }
}