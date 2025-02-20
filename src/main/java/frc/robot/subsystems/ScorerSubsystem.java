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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScorerConstants;

public class ScorerSubsystem extends SubsystemBase{
    private final SparkMax scorerRightMax;
    private final SparkMax scorerLeftMax;

    private final RelativeEncoder scorerRightEncoder;
    private final RelativeEncoder scorerLeftEncoder;

    private final SparkClosedLoopController scorerRightController;
    private final SparkClosedLoopController scorerLeftController;
    private double targetSetPoint = 0.0;

    private LaserCan laserCan_init;
    private LaserCan laserCan_end;


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
            laserCan_init.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan_init.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }        
        laserCan_end = new LaserCan(ScorerConstants.kEndLaserCANID);
        try {
            laserCan_end.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan_end.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan_end.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        scorerRightMax.configure(Configs.ScorerSubsystemConfigs.scorerRightMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        scorerLeftMax.configure(Configs.ScorerSubsystemConfigs.scorerLeftMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setScorerMaxLeft(double setPoint){
        scorerLeftMax.set(setPoint);

        targetSetPoint = setPoint;
    }
    public void setScorerMaxRight(double setPoint){
        scorerRightMax.set(setPoint);
        targetSetPoint = setPoint;
    }

    public boolean getProxStateInit(){
        LaserCan.Measurement measurementInit = laserCan_init.getMeasurement();
        double initDistance = measurementInit.distance_mm;
        if (measurementInit != null && measurementInit.status == laserCan_init.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return initDistance < ScorerConstants.kSensorProxDistanceMM;
        }
        else{
            return false;
        }
    }

    public boolean getProxStateEnd(){
        LaserCan.Measurement measurementEnd = laserCan_end.getMeasurement();
        double endDistance = measurementEnd.distance_mm;
        if (measurementEnd != null && measurementEnd.status == laserCan_init.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return endDistance < ScorerConstants.kSensorProxDistanceMM;
        }
        else{
            return false;
        }
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
        scorerRightController.setReference(0.35, ControlType.kDutyCycle);
        scorerLeftController.setReference(0.35, ControlType.kDutyCycle);
    }

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

    public boolean atSpeed() {
        return Math.abs(getScorerRightVelocityRPM() - targetSetPoint) < ScorerConstants.kScorerSpeedDeadbandRPM;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left scorer vel", scorerLeftEncoder.getVelocity());
        SmartDashboard.putNumber("right scorer vel", scorerRightEncoder.getVelocity());
    }
}
