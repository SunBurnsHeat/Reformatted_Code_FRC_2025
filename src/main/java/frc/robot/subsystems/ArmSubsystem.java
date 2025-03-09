package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.AbsoluteEncoderConfig;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    
    private final SparkMax armMax;
    private final SparkMax rollerMax;

    private final AbsoluteEncoder armMotorEncoder;
    private final RelativeEncoder rollerMotorEncoder;

    private final SparkClosedLoopController armMotorController;
    private final SparkClosedLoopController rollerMotorController;

    private Double targetPosition = 195.0;
    private double targetSetpoint = 0.0;


    public ArmSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        armMax = new SparkMax(ArmConstants.kArmMotorCANID, SparkMax.MotorType.kBrushless);
        rollerMax = new SparkMax(ArmConstants.kArmRollerMotorCANID, SparkMax.MotorType.kBrushless);

        armMotorEncoder = armMax.getAbsoluteEncoder();
        rollerMotorEncoder = rollerMax.getEncoder();

        armMotorController = armMax.getClosedLoopController();
        rollerMotorController = rollerMax.getClosedLoopController();

        armMax.configure(Configs.ArmSubsystemConfigs.armMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMax.configure(Configs.ArmSubsystemConfigs.rollerMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    }

    public void setArmRoller(double rollerSetpoint){ 
        // rollerMotorController.setReference(rollerSetpoint, ControlType.kDutyCycle);
        targetSetpoint = rollerSetpoint;
    }

    public void setArmPosition(double position){ // position is in degrees
        targetPosition = position;
    }

    public double getArmPosition(){
        return armMotorEncoder.getPosition();
    }

    public double getRollerVelocity(){
        return rollerMotorEncoder.getVelocity();
    }   

    public void stopArm(){
        armMotorController.setReference(0, ControlType.kDutyCycle);
        targetPosition = armMotorEncoder.getPosition();
    }

    public void stopRoller(){
        rollerMotorController.setReference(0, ControlType.kDutyCycle);
    }

    public boolean atPosition(){
        return Math.abs(getArmPosition() - targetPosition) < ArmConstants.kArmPositionDeadband;
    }

    public void incremPos(){
        if (targetPosition > ArmConstants.kFullExtendPosition) {
            targetPosition = targetPosition;
        }
        else{
            targetPosition += 5.0;
        }
    }

    public void decremPos(){
        if (targetPosition < ArmConstants.kArmReverseSoftLimit) {
            targetPosition = targetPosition;
        }
        else{
            targetPosition -= 5.0;
        }
    }

    @Override
    public void periodic(){

        // if(Math.abs(controller.getLeftY()) < 0.015) {
        //     setArmRoller(0);
        // }
        // else {
        //     setArmRoller(controller.getLeftY());
        // }
        // double velocity = ((targetPosition - getArmPosition())/0.01);
        // double FF = armFF.calculate(Units.degreesToRadians(targetPosition), velocity);    
        
        // if (!atPosition()) {
            armMotorController.setReference(targetPosition, ControlType.kPosition/* , ClosedLoopSlot.kSlot0 , FF*/);
        // }
        // else{
        //     armMotorController.setReference(0, ControlType.kDutyCycle);
        // }

        rollerMotorController.setReference(targetSetpoint, ControlType.kDutyCycle);

        SmartDashboard.putNumber("Arm Output", armMax.getAppliedOutput());
        SmartDashboard.putNumber("Arm Target Position", targetPosition);
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Arm at Position", atPosition());
    }

}
