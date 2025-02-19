package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScorerConstants;

public class ScorerSubsystem extends SubsystemBase{
    private final SparkMax scorerRightMax;
    private final SparkMax scorerLeftMax;

    private final RelativeEncoder scorerRightEncoder;
    private final RelativeEncoder scorerLeftEncoder;

    private final SparkClosedLoopController scorerRightController;
    private final SparkClosedLoopController scorerLeftController;
    private double targetSetPoint = 0.0;

    XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    public ScorerSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);
        scorerRightMax = new SparkMax(ScorerConstants.kScorerRightMotorCANID, MotorType.kBrushless);
        scorerLeftMax = new SparkMax(ScorerConstants.kScorerLeftMotorCANID, MotorType.kBrushless);

        scorerRightEncoder = scorerRightMax.getEncoder();
        scorerLeftEncoder = scorerLeftMax.getEncoder();

        scorerRightController = scorerRightMax.getClosedLoopController();
        scorerLeftController = scorerLeftMax.getClosedLoopController();

        scorerRightMax.configure(Configs.ScorerSubsystemConfigs.scorerRightMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        scorerLeftMax.configure(Configs.ScorerSubsystemConfigs.scorerLeftMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setScorerMax(double setPoint){
        scorerRightMax.set(setPoint);
        scorerLeftMax.set(setPoint);

        targetSetPoint = setPoint;
    }

    public void stop(){
        scorerRightController.setReference(0, ControlType.kDutyCycle);
        scorerLeftController.setReference(0, ControlType.kDutyCycle);

        targetSetPoint = 0;
    }

    public void setScorerVelocityRPM(double velocity){
        scorerRightController.setReference(velocity, ControlType.kVelocity);
        scorerLeftController.setReference(velocity, ControlType.kVelocity);

        targetSetPoint = velocity;
    }

    public double getScorerRightVelocityRPM(){
        return scorerRightEncoder.getVelocity();
    }

    public double getScorerLeftVelocityRPM(){
        return scorerLeftEncoder.getVelocity();
    }

    public void intake(){
        scorerRightController.setReference(ScorerConstants.kIntakeVelocityRPM, ControlType.kVelocity);
        scorerLeftController.setReference(ScorerConstants.kIntakeVelocityRPM, ControlType.kVelocity);

        targetSetPoint = ScorerConstants.kIntakeVelocityRPM;
    }

    public void ejectBottomRight(){
        scorerRightController.setReference(0.15, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.8, ControlType.kDutyCycle);
    }

    public void ejectBottomLeft(){
        scorerRightController.setReference(0.8, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.15, ControlType.kDutyCycle);
    }

    public void ejectElevated(){
        scorerRightController.setReference(0.7, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.7, ControlType.kDutyCycle);

    }

    public boolean atSpeed() {
        return Math.abs(getScorerRightVelocityRPM() - targetSetPoint) < ScorerConstants.kScorerSpeedDeadbandRPM;
    }

    @Override
    public void periodic() {
        if(Math.abs(controller.getLeftX()) < 0.015) {
            if (controller.getLeftX() >= 0.015) {
                ejectBottomRight();
            }
            else if (controller.getLeftX() <= -0.015) {
                ejectBottomLeft();
            }
        }
    }
}
