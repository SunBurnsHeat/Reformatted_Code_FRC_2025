package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.XboxController;
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

    // private final XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    private final ProfiledPIDController leadElevatorProfiledPIDController;
    private final ElevatorFeedforward elevatorFF;

    private SparkClosedLoopController leadElevatorController;

    private Double targetPosition = 1.25;
    
    public ElevatorSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        leadElevatorMax = new SparkMax(ElevatorConstants.kLeadElevatorMotorCANID, MotorType.kBrushless);
        followElevatorMax = new SparkMax(ElevatorConstants.kFollowElevatorMotorCANID, MotorType.kBrushless);

        leadElevatorEncoder = leadElevatorMax.getEncoder();

        // ShuffleboardTab tab = Shuffleboard.getTab("Robot Controls");
        // elevatorSetpoint = tab.add("Elevator Power", 0.0)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withProperties(Map.of("min", -1, "max", 1))
        // .getEntry();

        leadElevatorController = leadElevatorMax.getClosedLoopController();

        leadElevatorMax.configure(Configs.ElevatorSubsystemConfigs.leadElevatorMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followElevatorMax.configure(Configs.ElevatorSubsystemConfigs.followElevatorMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leadElevatorEncoder.setPosition(0);

        leadElevatorProfiledPIDController = new ProfiledPIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.elevConstraints, ElevatorConstants.kDt);
        elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        leadElevatorProfiledPIDController.setTolerance(ElevatorConstants.kElevatorHeightDeadbandInches);
    }

    public void setPosition(double position){

        // leadElevatorProfiledPIDController.setGoal(position);
        // leadElevatorMax.setVoltage(
        // leadElevatorProfiledPIDController.calculate(leadElevatorEncoder.getPosition())
        //     + elevatorFF.calculate(leadElevatorProfiledPIDController.getSetpoint().velocity));

        // double velocity = ((position - getPosition())/0.02);
        // double FF = elevatorFF.calculate(velocity);

        // leadElevatorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);

        // leadElevatorController.setReference(position, ControlType.kPosition);
        targetPosition = position;
    }


    public double getPosition(){
        return leadElevatorEncoder.getPosition();
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

    public void setLeadElevPosition(double position){
        leadElevatorEncoder.setPosition(position);
    }

    public boolean atHeight(){
        return Math.abs(getPosition() - targetPosition) < ElevatorConstants.kElevatorHeightDeadbandInches;
    }

    public void incremPos(){
        if (targetPosition > ElevatorConstants.kElevatorForwardSoftLimit) {
            targetPosition = targetPosition;
        }
        else{
            targetPosition += 1.5;
        }
    }

    public void decremPos(){
        if (targetPosition < ElevatorConstants.kElevatorReverseSoftLimit) {
            targetPosition = targetPosition;
        }
        targetPosition -= 1.5;
    }

    public boolean atDangerHeight(){
        return getPosition() > ElevatorConstants.kElevatorPosition_L2;
    }

    @Override
    public void periodic(){
        // // double elevatorTarget = elevatorSetpoint.getDouble(0.0);
        // if(Math.abs(controller.getLeftY()) < 0.015) {
        //     setElevator(0.0);
        // }
        // else {
        //     setElevator(controller.getLeftY()*(-1));
        // }
            if (!atHeight()) {
                // leadElevatorProfiledPIDController.setGoal(targetPosition);
                // leadElevatorMax.setVoltage(
                // leadElevatorProfiledPIDController.calculate(leadElevatorEncoder.getPosition())
                //     + elevatorFF.calculate(leadElevatorProfiledPIDController.getSetpoint().velocity));    

                leadElevatorController.setReference(targetPosition, ControlType.kPosition);
            }
            // else{
            //     leadElevatorController.setReference(0, ControlType.kDutyCycle);
            // }

        SmartDashboard.putNumber("Elevator Pos", getPosition());
        SmartDashboard.putBoolean("Elevator At Pos", atHeight());
        SmartDashboard.putNumber("Elevator target position", targetPosition);

    }
}
