package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ScorerConstants;

public class ScorerSubsystem extends SubsystemBase{
    private final SparkMax scorerRightMax;
    private final SparkMax scorerLeftMax;

    private final RelativeEncoder scorerRightEncoder;
    private final RelativeEncoder scorerLeftEncoder;

    private final SparkClosedLoopController scorerRightController;
    private final SparkClosedLoopController scorerLeftController;

    private LaserCan laserCan_init;
    private LaserCan laserCan_end;

    public boolean initProx;
    public boolean endProx;

    private double initValue;
    private double endValue;


    public ScorerSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);
        scorerRightMax = new SparkMax(ScorerConstants.kScorerRightMotorCANID, MotorType.kBrushless);
        scorerLeftMax = new SparkMax(ScorerConstants.kScorerLeftMotorCANID, MotorType.kBrushless);

        scorerRightEncoder = scorerRightMax.getEncoder();
        scorerLeftEncoder = scorerLeftMax.getEncoder();

        scorerRightController = scorerRightMax.getClosedLoopController();
        scorerLeftController = scorerLeftMax.getClosedLoopController();

        laserCan_init = new LaserCan(ScorerConstants.kInitLaserCANID);
        try {
            laserCan_init.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan_init.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 14));
            laserCan_init.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }        
        laserCan_end = new LaserCan(ScorerConstants.kEndLaserCANID);
        try {
            laserCan_end.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan_end.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 8));
            laserCan_end.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        scorerRightMax.configure(Configs.ScorerSubsystemConfigs.scorerRightMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        scorerLeftMax.configure(Configs.ScorerSubsystemConfigs.scorerLeftMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setScorerMaxLeft(double setPoint){
        scorerLeftController.setReference(setPoint, ControlType.kDutyCycle);
    }
    public void setScorerMaxRight(double setPoint){
        scorerRightController.setReference(setPoint, ControlType.kDutyCycle);
    }

    public boolean getProxStateInit(){
        LaserCan.Measurement measurementInit = laserCan_init.getMeasurement();
        double initDistance = measurementInit.distance_mm;
        if (measurementInit != null && measurementInit.status == laserCan_init.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return initDistance < 40;
        }
        else{
            return false;
        }
    }

    public boolean getProxStateEnd(){
        LaserCan.Measurement measurementEnd = laserCan_end.getMeasurement();
        double endDistance = measurementEnd.distance_mm;
        if (measurementEnd != null && measurementEnd.status == laserCan_init.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return endDistance < 40;
        }
        else{
            return false;
        } 
    }

    public void stop(){
        scorerRightController.setReference(0, ControlType.kDutyCycle);
        scorerLeftController.setReference(0, ControlType.kDutyCycle);
    }

    public void setScorerVelocityRPM(double velocity){
        scorerRightController.setReference(velocity, ControlType.kVelocity);
        scorerLeftController.setReference(velocity, ControlType.kVelocity);
    }

    public double getScorerRightVelocityRPM(){
        return scorerRightEncoder.getVelocity();
    }

    public boolean holdingCoral(){
        return (!initProx && endProx);
    }

    public boolean ongoingCoral(){
        return (initProx && endProx);
    }

    public boolean hasCoral(){
        return (initProx && !endProx);
    }

    public boolean notHasCoral(){
        return (!initProx && !endProx);
    }
 

    public double getScorerLeftVelocityRPM(){
        return scorerLeftEncoder.getVelocity();
    }

    // public void intake(){
        
    //     double kDefaultSpeed = 0.20;
    //     double kSlowSpeed = 0.12;
    //     double kBrake = 0.0;

    //     boolean currentInitProx = getProxStateInit();
    //     boolean currentEndProx = getProxStateEnd();
    //         if (!getProxStateInit() && getProxStateEnd()) {
    //             scorerRightController.setReference(0, ControlType.kDutyCycle);
    //             scorerLeftController.setReference(0, ControlType.kDutyCycle);
                
    //             if (getProxStateInit() && !getProxStateEnd()) {
    //                 scorerRightController.setReference(.3, ControlType.kDutyCycle);
    //                 scorerLeftController.setReference(.3, ControlType.kDutyCycle);

    //                 if((!getProxStateInit() && !getProxStateEnd()) || (getProxStateInit() && getProxStateEnd())){
    //                     scorerRightController.setReference(.3, ControlType.kDutyCycle);
    //                     scorerLeftController.setReference(.3, ControlType.kDutyCycle);
    //                 }

    //             }
    //         }
    // }

    public void stopScorer(){
        scorerRightController.setReference(0, ControlType.kDutyCycle);
        scorerLeftController.setReference(0, ControlType.kDutyCycle);
    }

    public void ejectBottomRight(){
        scorerRightController.setReference(0.05, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.3, ControlType.kDutyCycle);
    }

    public void ejectBottomLeft(){
        scorerRightController.setReference(0.3, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.05, ControlType.kDutyCycle);
    }

    public void ejectElevated(){
        scorerRightController.setReference(0.25, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.25, ControlType.kDutyCycle);

    }

    @Override
    public void periodic() {
        initProx = getProxStateInit();
        endProx = getProxStateEnd();

        // if (getIntakable()) {

        //     // if (getProxStateEnd()) {
        //         scorerRightController.setReference(0.1, ControlType.kDutyCycle);
        //         scorerLeftController.setReference(0.1, ControlType.kDutyCycle);
        //     // }

        //     // else{
        //     //     scorerRightController.setReference(0.2, ControlType.kDutyCycle);
        //     //     scorerLeftController.setReference(0.2, ControlType.kDutyCycle);

        //     // }
        // }

        // else{
        //     // scorerRightController.setReference(0, ControlType.kDutyCycle);
        //     // scorerLeftController.setReference(0, ControlType.kDutyCycle);
        // }

        // SmartDashboard.putNumber("left scorer output", scorerLeftMax.getAppliedOutput());
        // SmartDashboard.putNumber("right scorer output", scorerRightMax.getAppliedOutput());
        SmartDashboard.putBoolean("init Prox State", initProx);
        SmartDashboard.putBoolean("end Prox State", endProx);

        // if (!hasCoral()) {
        //     LedSubsystem.setAllianceSolid();
        // }
    }
}
