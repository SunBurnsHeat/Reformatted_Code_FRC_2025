package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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

    private final XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    private final RelativeEncoder leadElevatorEncoder;

    private final ElevatorFeedforward elevatorFF;

    private final SparkClosedLoopController leadElevatorController;
    private double targetPosition;
    
    public ElevatorSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        leadElevatorMax = new SparkMax(ElevatorConstants.kLeadElevatorMotorCANID, MotorType.kBrushless);
        followElevatorMax = new SparkMax(ElevatorConstants.kFollowElevatorMotorCANID, MotorType.kBrushless);

        leadElevatorEncoder = leadElevatorMax.getEncoder();

        leadElevatorController = leadElevatorMax.getClosedLoopController();

        // leadElevatorMax.configure(Configs.ElevatorSubsystemConfigs.leadElevatorMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // followElevatorMax.configure(Configs.ElevatorSubsystemConfigs.followElevatorMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leadElevatorEncoder.setPosition(0);
        targetPosition = 0.0;

        elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    }

    public void setHeight(double position){
        double velocity = ((position - getHeight())/0.02);
        double FF = elevatorFF.calculate(velocity);

        leadElevatorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);
        position = targetPosition;
    }

    public double getHeight(){
        return leadElevatorEncoder.getPosition();
    }

    public void setElevatorSpeed(double speed){
        leadElevatorController.setReference(speed, ControlType.kVelocity);
    }

    public void setElevator(double setpoint){
        leadElevatorController.setReference(setpoint, ControlType.kDutyCycle);
    }

    public double getElevatorSpeed(){
        return leadElevatorEncoder.getVelocity();
    }

    public void stopElevator(){
        leadElevatorMax.set(0);
        followElevatorMax.set(0);
        targetPosition = leadElevatorEncoder.getPosition();
    }

    public boolean atHeight(){
        return Math.abs(getHeight() - targetPosition) < ElevatorConstants.kElevatorHeightDeadbandRaw;
    }

    // @Override
    // public void periodic(){
    //     SmartDashboard.putNumber("Elevator Pos", getHeight());
    //     SmartDashboard.putBoolean("Elevator At Pos", atHeight());
    //     if(Math.abs(controller.getRightY()) > 0.15){
    //        setElevator(controller.getRightY()*10);
    //     }
    //     else{
    //         setElevator(0);
    //     }
    // }
}
