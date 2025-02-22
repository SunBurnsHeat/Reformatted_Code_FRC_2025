package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

public class ArmSubsystem extends SubsystemBase{
    
    private final SparkMax armMax = new SparkMax(ArmConstants.kArmMotorCANID, SparkMax.MotorType.kBrushless);
    private final SparkMax rollerMax = new SparkMax(ArmConstants.kArmRollerMotorCANID, SparkMax.MotorType.kBrushless);

    private final RelativeEncoder armMotorEncoder;
    private final RelativeEncoder rollerMotorEncoder;

    private final SparkClosedLoopController armMotorController;
    private final SparkClosedLoopController rollerMotorController;

    private Double targetPosition;
    private Double targetSetpoint;

    private final XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    public ArmSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        armMotorEncoder = armMax.getEncoder();
        rollerMotorEncoder = rollerMax.getEncoder();

        armMotorController = armMax.getClosedLoopController();
        rollerMotorController = rollerMax.getClosedLoopController();

        armMax.configure(Configs.ArmSubsystemConfigs.armMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMax.configure(Configs.ArmSubsystemConfigs.rollerMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armMotorEncoder.setPosition(0);

        targetPosition = 0.0;
        targetSetpoint = 0.0;
    }

    public void setArm(double targetSetPoint){
        armMotorController.setReference(targetSetPoint, ControlType.kDutyCycle);
    }

    public void setArmRoller(double speed){ // speed is in RPM
        targetSetpoint = speed;
        rollerMotorController.setReference(speed, ControlType.kVelocity);
    }

    public void setArmPosition(double position){ // position is in degrees
        targetPosition = position;
        armMotorController.setReference(position, ControlType.kPosition);
    }

    public double getArmPosition(){
        return armMotorEncoder.getPosition();
    }

    public double getRollerVelocity(){
        return rollerMotorEncoder.getVelocity();
    }   

    public void stopArm(){
        armMotorController.setReference(0, ControlType.kDutyCycle);
        targetPosition = null;
    }

    public void stopRoller(){
        rollerMotorController.setReference(0, ControlType.kDutyCycle);
        targetSetpoint = null;
    }

    public void stow(){
        armMotorController.setReference(ArmConstants.kStowPosition, ControlType.kPosition);
    }

    public void fullExtend(){
        armMotorController.setReference(ArmConstants.kFullExtendPosition, ControlType.kPosition);
    }

    public void partialExtend(){
        armMotorController.setReference(ArmConstants.kPartialPosition, ControlType.kPosition);
    }

    public boolean atPosition(){
        return Math.abs(getArmPosition() - targetPosition) < ArmConstants.kArmPositionDeadband;
    }

    public boolean atSpeed(){
        return Math.abs(getRollerVelocity() - targetSetpoint) < ArmConstants.kArmRollerSpeedDeadband;
    }

    @Override
    public void periodic(){

        if(Math.abs(controller.getLeftY()) < 0.015) {
            setArmRoller(0);
        }
        else {
            setArmRoller(controller.getLeftY());
        }

        SmartDashboard.putBoolean("Arm at Position", atPosition());
        SmartDashboard.putBoolean("Roller at Speed", atSpeed());
    }

}
