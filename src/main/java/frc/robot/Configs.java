package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.Constants.WinchConstants;

public final class Configs {
    public static final class MAXSwereveModule{
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static{
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }

    }

    public static final class ElevatorSubsystemConfigs{
        public static final SparkMaxConfig leadElevatorMaxConfig = new SparkMaxConfig();
        public static final SparkMaxConfig followElevatorMaxConfig = new SparkMaxConfig();


        static{
                leadElevatorMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                leadElevatorMaxConfig.encoder
                        .positionConversionFactor(2.0*Math.PI*ElevatorConstants.kElevatorSpraketDiameterInches / ElevatorConstants.kElevatorMotorGearRatio)
                        .velocityConversionFactor(2.0*Math.PI*ElevatorConstants.kElevatorSpraketDiameterInches / ElevatorConstants.kElevatorMotorGearRatio / 60.0);
                leadElevatorMaxConfig.closedLoop
                        .maxMotion.maxVelocity(ElevatorConstants.kMaxVelocity)
                        .maxAcceleration(ElevatorConstants.kMaxAcceleration);
                leadElevatorMaxConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.145, 2e-5, 0)
                        .iZone(0.2)
                        .velocityFF(0.0)
                        .outputRange(-0.2, 0.2);
                leadElevatorMaxConfig.softLimit
                        .forwardSoftLimit(47.5)
                        .forwardSoftLimitEnabled(true)
                        .reverseSoftLimit(0)
                        .reverseSoftLimitEnabled(true);
                leadElevatorMaxConfig.limitSwitch
                        .forwardLimitSwitchEnabled(true)
                        .forwardLimitSwitchType(Type.kNormallyOpen)
                        .reverseLimitSwitchEnabled(true)
                        .reverseLimitSwitchType(Type.kNormallyOpen);
                

                followElevatorMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12)
                        .follow(ElevatorConstants.kLeadElevatorMotorCANID, true);
                followElevatorMaxConfig.closedLoop.maxMotion
                        .maxVelocity(ElevatorConstants.kMaxVelocity)
                        .maxAcceleration(ElevatorConstants.kMaxAcceleration);
                }

    }

    public static final class ScorerSubsystemConfigs{
        public static final SparkMaxConfig scorerRightMaxConfig = new SparkMaxConfig();
        public static final SparkMaxConfig scorerLeftMaxConfig = new SparkMaxConfig();


        static{
                scorerRightMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(true)
                        .smartCurrentLimit(30)
                        .voltageCompensation(12);
                scorerRightMaxConfig.encoder
                        .positionConversionFactor(1.0 / ScorerConstants.kScorerMotorGearRatio)
                        .velocityConversionFactor(1.0 / ScorerConstants.kScorerMotorGearRatio / 60.0);
                scorerRightMaxConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.00125, 0, 0)
                        .outputRange(-.5, .5);

                scorerLeftMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(false)
                        .smartCurrentLimit(30)
                        .voltageCompensation(12);
                scorerLeftMaxConfig.encoder
                        .positionConversionFactor(1.0 / ScorerConstants.kScorerMotorGearRatio)
                        .velocityConversionFactor(1.0 / ScorerConstants.kScorerMotorGearRatio / 60.0);
                scorerLeftMaxConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.00115, 0, 0)
                        .outputRange(-.5, .5);
                }
        }

        public static final class ArmSubsystemConfigs{

        public static final SparkMaxConfig armMaxConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rollerMaxConfig = new SparkMaxConfig();

        static{
                armMaxConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                armMaxConfig.absoluteEncoder
                        .inverted(true)
                        .positionConversionFactor(360)
                        .velocityConversionFactor(360 / 60.0);
                armMaxConfig.softLimit
                        .forwardSoftLimit(ArmConstants.kArmForwardSoftLimit)
                        .forwardSoftLimitEnabled(true)
                        .reverseSoftLimit(ArmConstants.kArmReverseSoftLimit)
                        .reverseSoftLimitEnabled(true);
                // armMaxConfig.closedLoop
                //         .maxMotion.maxVelocity(2.5)
                //         .maxAcceleration(1.25);
                armMaxConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // set to absolute encoder
                        .pid(0.015, 0e-4, 0)
                        .iZone(0.0)
                        .velocityFF(0.0)
                        .outputRange(-.45, .45);

                rollerMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(30)
                        .voltageCompensation(12);
                rollerMaxConfig.encoder
                        .positionConversionFactor(1.0 / ArmConstants.kArmRollerMotorGearRatio)
                        .velocityConversionFactor(1.0 / ArmConstants.kArmRollerMotorGearRatio / 60.0);
                rollerMaxConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.00115, 0, 0)
                        .outputRange(-.6, .6);
                }
        }

        public static final class WinchConfigs {
        
        public static final SparkMaxConfig winchMaxConfig = new SparkMaxConfig();
        public static final SparkMaxConfig trapMaxConfig = new SparkMaxConfig();

        static{
                winchMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                winchMaxConfig.softLimit
                        .forwardSoftLimit(0)
                        .forwardSoftLimitEnabled(true)
                        .reverseSoftLimit(WinchConstants.kTopPosition)
                        .reverseSoftLimitEnabled(true);
                winchMaxConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.05, 2e-5, 0.0)
                        .iZone(0.1)
                        .outputRange(-1, 1);

                trapMaxConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(25)
                        .voltageCompensation(12);
                trapMaxConfig.limitSwitch
                        .reverseLimitSwitchEnabled(true)
                        .reverseLimitSwitchType(Type.kNormallyOpen);
                }
        }
}
