package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;

public class RobotControlShuffleboard extends SubsystemBase {
    private GenericEntry elevatorSetpoint;
    private GenericEntry armAngleSetpoint;
    private GenericEntry rollerSpeedSetpoint;
    private GenericEntry winchSpeedSetpoint;

    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;  // Arm and rollers are together
    private WinchSubsystem winch;

    public RobotControlShuffleboard(ElevatorSubsystem elevator, ArmSubsystem arm, WinchSubsystem winch) {
        this.elevator = elevator;
        this.arm = arm;
        this.winch = winch;
        
        setupShuffleboard();
    }

    private void setupShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Robot Controls");

        elevatorSetpoint = tab.add("Elevator Power", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

        armAngleSetpoint = tab.add("Arm Angle", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", 0, "max", 150))
            .getEntry();

        rollerSpeedSetpoint = tab.add("Roller Speed", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1, "max", 1)) // Full speed range
            .getEntry();

        winchSpeedSetpoint = tab.add("Winch Speed", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

        tab.add("Elevator Position", elevator.getPosition());
        tab.add("Arm Position", arm.getArmPosition());
        tab.add("Winch Position", winch.getPosition());
    }

    @Override
    public void periodic() {
        double elevatorTarget = elevatorSetpoint.getDouble(0.0);
        double armTarget = armAngleSetpoint.getDouble(0.0);
        double rollerSpeed = rollerSpeedSetpoint.getDouble(0.0);
        double winchSpeed = winchSpeedSetpoint.getDouble(0.0);

        // Apply values to subsystems
        elevator.setPosition(elevatorTarget);
        arm.setArmPosition(armTarget);
        arm.setArmRoller(rollerSpeed);
        winch.setWinch(winchSpeed);

        // Update SmartDashboard with current sensor values
        SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("Arm Position", arm.getArmPosition());
        SmartDashboard.putNumber("Arm Roller Position", arm.getRollerVelocity());
        SmartDashboard.putNumber("Winch Position", winch.getPosition());
    }
}