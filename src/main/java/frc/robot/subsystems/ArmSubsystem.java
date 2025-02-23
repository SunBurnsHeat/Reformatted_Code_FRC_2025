package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

public class ArmSubsystem extends SubsystemBase{
    
    private final SparkMax armMax;
    private final SparkMax rollerMax;

    private final RelativeEncoder armMotorEncoder;
    private final RelativeEncoder rollerMotorEncoder;

    private ArmFeedforward armFF;

    private final SparkClosedLoopController armMotorController;
    private final SparkClosedLoopController rollerMotorController;

    private double targetPosition = 0.0;
    private double targetSetpoint = 0.0;

    // private final XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    public ArmSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        armMax = new SparkMax(ArmConstants.kArmMotorCANID, SparkMax.MotorType.kBrushless);
        rollerMax = new SparkMax(ArmConstants.kArmRollerMotorCANID, SparkMax.MotorType.kBrushless);

        armMotorEncoder = armMax.getEncoder();
        rollerMotorEncoder = rollerMax.getEncoder();

        armMotorController = armMax.getClosedLoopController();
        rollerMotorController = rollerMax.getClosedLoopController();

        armMax.configure(Configs.ArmSubsystemConfigs.armMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMax.configure(Configs.ArmSubsystemConfigs.rollerMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armMotorEncoder.setPosition(0);

        armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    }

    public void setArmRoller(double rollerSetpoint){ // speed is in RPM
        targetSetpoint = rollerSetpoint;
        rollerMotorController.setReference(rollerSetpoint, ControlType.kDutyCycle);
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

    public boolean atSpeed(){
        return Math.abs(getRollerVelocity() - targetSetpoint) < ArmConstants.kArmRollerSpeedDeadband;
    }

    @Override
    public void periodic(){

        // if(Math.abs(controller.getLeftY()) < 0.015) {
        //     setArmRoller(0);
        // }
        // else {
        //     setArmRoller(controller.getLeftY());
        // }
        double velocity = ((targetPosition - getArmPosition())/0.01);
        double FF = armFF.calculate(Units.degreesToRadians(targetPosition), velocity);    
        
        if (!atPosition()) {
            armMotorController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);
        }
        else{
            armMotorController.setReference(0, ControlType.kDutyCycle);
        }

        SmartDashboard.putNumber("Arm Target Position", targetPosition);
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Arm at Position", atPosition());
        SmartDashboard.putBoolean("Roller at Speed", atSpeed());
    }

}
