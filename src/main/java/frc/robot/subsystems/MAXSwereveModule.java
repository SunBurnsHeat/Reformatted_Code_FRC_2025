package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;


public class MAXSwereveModule {

    // create variables for modules
    private final SparkFlex kDrivingFlex;
    private final SparkMax kTurningMAX;

    // create variables for encoders
    private final RelativeEncoder kDriveEncoder;
    private final AbsoluteEncoder kTurningEncoder;

    // create variables for PIDcontrollers
    private final SparkClosedLoopController kDrivingClosedLoopController;
    private final SparkClosedLoopController kTurningClosedLoopController;

    // initial value for chassis offset
    private double chassisoffset = 0.0;
    // create variable for the state of the module
    private SwerveModuleState targetstate = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwereveModule(int turningCANid, int drivingCANid, double angleOffset) {
        // initiate the module motors
        kTurningMAX = new SparkMax(turningCANid, MotorType.kBrushless);
        kDrivingFlex = new SparkFlex(drivingCANid, MotorType.kBrushless);


        // initiate the module encoders
        kDriveEncoder = kDrivingFlex.getEncoder();
        kTurningEncoder = kTurningMAX.getAbsoluteEncoder();
        
        // initiate the closed-loop-controllers
        kTurningClosedLoopController = kTurningMAX.getClosedLoopController();
        kDrivingClosedLoopController = kDrivingFlex.getClosedLoopController();

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        kDrivingFlex.configure(Configs.MAXSwereveModule.drivingConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kTurningMAX.configure(Configs.MAXSwereveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        chassisoffset = angleOffset;
        targetstate.angle = new Rotation2d(kTurningEncoder.getPosition());
        kDriveEncoder.setPosition(0);
        
    }

    public SwerveModuleState getState() {
        // method that returns the current state value of swerve
        // uses drive encoder to get velocity
        // uses the turning encoder to get the position after being adjusted by the chassis offset
        // returns velocity and angle
        return new SwerveModuleState(kDriveEncoder.getVelocity(), new Rotation2d(kTurningEncoder.getPosition() - chassisoffset));
    }

    public SwerveModulePosition getPosition() {
        // method that returns the current position of swerve
        // uses the drive encoder to get the position
        // uses the turning encoder to the position after being adjusted by the chassis offset
        // returns poosition and angle
        return new SwerveModulePosition(kDriveEncoder.getPosition(), new Rotation2d(kTurningEncoder.getPosition() - chassisoffset));
    }

    // sets the desired state of the module
    public void setState(SwerveModuleState tState){

        // creates a new swerve module state that holds the adjusted state
        SwerveModuleState correctState = new SwerveModuleState();

        // set to target speed from "tState"
        correctState.speedMetersPerSecond = tState.speedMetersPerSecond;
        // added chassis offset
        correctState.angle = tState.angle.plus(Rotation2d.fromRadians(chassisoffset));

        // optimize with minimal movement
        correctState.optimize(new Rotation2d(kTurningEncoder.getPosition()));

        // sets desired speed of driving motor and angle for turning motor with the PID controllers
        kDrivingClosedLoopController.setReference(correctState.speedMetersPerSecond, ControlType.kVelocity);
        kTurningClosedLoopController.setReference(correctState.angle.getRadians(), ControlType.kPosition); // there is no such thing as angle controltype

        // updated the target state
        targetstate = correctState;
    }

    // New method for SysID: Apply raw voltage to drive motor
    public void setDriveVoltage(double voltage) {
        kDrivingFlex.setVoltage(voltage); // Bypasses PID
        // Lock steering at 0 degrees (relative to chassis offset)
        kTurningClosedLoopController.setReference(chassisoffset, ControlType.kPosition);
    }

    // New method for SysID: Get applied voltage
    public double getDriveVoltage() {
        return kDrivingFlex.getAppliedOutput() * kDrivingFlex.getBusVoltage();
    }

    // resets the drive encoder position to zero
    public void resetEncoder() {
        kDriveEncoder.setPosition(0);
        // the turning encoder is not resetted because it is important to keep the turning state offset acknowledged
    }
}
