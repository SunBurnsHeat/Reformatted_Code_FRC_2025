package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.WinchConfigs;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase{
    private final SparkMax winchMax;
    private final SparkMax trapMax;

    public static int trapCounter;

    private final RelativeEncoder winchEncoder;

    
    private final XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    public WinchSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        winchMax = new SparkMax(WinchConstants.kWinchCANID, MotorType.kBrushless);
        trapMax = new SparkMax(WinchConstants.kTrapCANID, MotorType.kBrushed);
        winchEncoder = winchMax.getEncoder();

        winchMax.configure(WinchConfigs.winchMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchEncoder.setPosition(0);
        
        trapCounter = 0;
    }

    public void setWinch(double setPoint){
        winchMax.set(setPoint);
    }

    public double getPosition(){
        return winchEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if (trapCounter > 0) {
            if(Math.abs(controller.getLeftY()) < 0.015) {
                setWinch(0.0);
            }
            else {
                setWinch(-controller.getLeftY()*WinchConstants.kWinchSpeed);
            }
        }

        SmartDashboard.putNumber("Winch Pos", getPosition());
    }
// 224
    public void openTrap(){
        trapMax.set(WinchConstants.trapOpenSpeed);
    }

    public void closeTrap(){
        trapMax.set(WinchConstants.trapCloseSpeed);
    }

    public void stopTrap(){
        trapMax.set(0);
    }
}
