package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leadElevatorMax;
    private final SparkMax followElevatorMax;
    private final RelativeEncoder leadElevatorEncoder;

    private GenericEntry elevatorSetpoint;


    private final ProfiledPIDController leadElevatorProfiledPIDController;
    private final ElevatorFeedforward elevatorFF;

    private final SparkClosedLoopController leadElevatorController;

    private double targetPosition;
    
    public ElevatorSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        leadElevatorMax = new SparkMax(ElevatorConstants.kLeadElevatorMotorCANID, MotorType.kBrushless);
        followElevatorMax = new SparkMax(ElevatorConstants.kFollowElevatorMotorCANID, MotorType.kBrushless);

        leadElevatorEncoder = leadElevatorMax.getEncoder();

        ShuffleboardTab tab = Shuffleboard.getTab("Robot Controls");
        elevatorSetpoint = tab.add("Elevator Power", 0.0)
        .withWidget("Number Slider")
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

        leadElevatorController = leadElevatorMax.getClosedLoopController();

        leadElevatorMax.configure(Configs.ElevatorSubsystemConfigs.leadElevatorMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followElevatorMax.configure(Configs.ElevatorSubsystemConfigs.followElevatorMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leadElevatorEncoder.setPosition(0);
        targetPosition = 0.0;

        leadElevatorProfiledPIDController = new ProfiledPIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.elevConstraints, ElevatorConstants.kDt);
        elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    }

    public void setPosition(double position){

        // leadElevatorProfiledPIDController.setGoal(position);
        // leadElevatorMax.setVoltage(
        // leadElevatorProfiledPIDController.calculate(leadElevatorEncoder.getPosition())
        //     + elevatorFF.calculate(leadElevatorProfiledPIDController.getSetpoint().velocity));

        // double velocity = ((position - getPosition())/0.02);
        // double FF = elevatorFF.calculate(velocity);

        // leadElevatorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);

        leadElevatorController.setReference(position, ControlType.kPosition);
        position = targetPosition;
    }

    public double getPosition(){
        return leadElevatorEncoder.getPosition();
    }

    public void setElevatorSpeed(double speed){
        leadElevatorController.setReference(speed, ControlType.kVelocity);
    }

    public double getElevatorSpeed(){
        return leadElevatorEncoder.getVelocity();
    }


    public void setElevator(double setpoint){
        leadElevatorController.setReference(setpoint, ControlType.kDutyCycle);
    }

    public void stopElevator(){
        leadElevatorMax.set(0);
        followElevatorMax.set(0);
        targetPosition = leadElevatorEncoder.getPosition();
    }

    public boolean atHeight(){
        return Math.abs(getPosition() - targetPosition) < ElevatorConstants.kElevatorHeightDeadbandRaw;
    }

    @Override
    public void periodic(){
        double elevatorTarget = elevatorSetpoint.getDouble(0.0);


        SmartDashboard.putNumber("Elevator Pos", getPosition());
        SmartDashboard.putBoolean("Elevator At Pos", atHeight());
    }
}
