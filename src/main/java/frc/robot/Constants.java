// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveConstants {
        public static final int kFrontLeftDriveCANID = 1;
        public static final int kFrontRightDriveCANID = 3;
        public static final int kBackLeftDriveCANID = 5;
        public static final int kBackRightDriveCANID = 7;

        public static final int kFrontLeftSteerCANID = 2;
        public static final int kFrontRightSteerCANID = 4;
        public static final int kBackLeftSteerCANID = 6;
        public static final int kBackRightSteerCANID = 8;

        public static final double rotationSlewRate = 2.0;
        public static final double directionSlewRate = 1.2;
        public static final double magLimiterSlewRate = 1.8;

        public static final double kTrackWidth = Units.inchesToMeters(29);
        public static final double kTrackLength = Units.inchesToMeters(29);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kTrackLength / 2, kTrackWidth / 2),
                new Translation2d(kTrackLength / 2, -kTrackWidth / 2),
                new Translation2d(-kTrackLength / 2, kTrackWidth / 2),
                new Translation2d(-kTrackLength / 2, -kTrackWidth / 2));

        public static final double kFrontLeftOffset = (-Math.PI / 2);
        public static final double kFrontRightOffset = 0;
        public static final double kBackLeftOffset = Math.PI;
        public static final double kBackRightOffset = (Math.PI / 2);

        public static final double kMaxSpeedMetersPerSec = 4.0; // max speed in mps
        public static final double kMaxAngSpeedRadiansPerSec = 2 * Math.PI; // max turning speed in rps
    }

    public static class ModuleConstants {
        public static final int kDrivingMotorPinionTeeth = 13;

        public static final boolean kTurningEncoderInverted = true;

        public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762*2.8/3.3;
        public static final double kWheelRadiusMeters = kWheelDiameterMeters/2;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoPilotControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class NeoVortexMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class LEDConstants {
        public static final int kLEDBarPWM = 9;
        public static final int ledLength = 40;
        public static final int ledBufferLength = 40;
    }

    public static final class AutoConstants {

            public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                74.088,         // Mass from "robotMass"
                6.883, // MOI from "robotMOI"
                new ModuleConfig(
                ModuleConstants.kWheelRadiusMeters,              // Motor type from "driveMotorType"
                NeoVortexMotorConstants.kFreeSpeedRpm,                // Gearing ratio from "driveGearing"
                1.3,                // Wheel radius (m) from "driveWheelRadius"
                DCMotor.getNeoVortex(1), // Max velocity (m/s)
                40,                 // Current limit (amps) from "driveCurrentLimit"
                4                  // Coefficient of friction from "wheelCOF"
                ),                       // Module config
                new Translation2d[] {
                    new Translation2d(0.273, 0.273),  // FL
                    new Translation2d(0.273, -0.273), // FR
                    new Translation2d(-0.273, 0.273), // BL
                    new Translation2d(-0.273, -0.273) // BR
                }                       // Module offsets
            );

        public static final double kMaxSpeedMetersPerSecondStandard = 2.25;
        public static final double kMaxAccelerationMetersPerSecondSquaredStandard = 1.75;

        public static final double kMaxSpeedMetersPerSecondFast = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquaredFast = 2.25;

        public static final double kMaxSpeedMetersPerSecondSlow = 1.75;
        public static final double kMaxAccelerationMetersPerSecondSquaredSlow = 1.45;

        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final TrajectoryConfig kTrajConfigStandard = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecondStandard, AutoConstants.kMaxAccelerationMetersPerSecondSquaredStandard)
            .setKinematics(DriveConstants.kDriveKinematics);
        
        public static final TrajectoryConfig kTrajConfigFast = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecondFast, AutoConstants.kMaxAccelerationMetersPerSecondSquaredFast)
            .setKinematics(DriveConstants.kDriveKinematics);

        public static final TrajectoryConfig kTrajConfigSlow = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecondSlow/2, AutoConstants.kMaxAccelerationMetersPerSecondSquaredSlow/2)
            .setKinematics(DriveConstants.kDriveKinematics);

        public static final TrajectoryConfig kTrajConfigStandardReverse = kTrajConfigStandard.setReversed(true);

        public static final TrajectoryConfig kTrajConfigFastReverse = kTrajConfigFast.setReversed(true);

        public static final TrajectoryConfig kTrajConfigSlowReverse = kTrajConfigSlow.setReversed(true);
    }

    public static final class ElevatorConstants{
        public static final int kLeadElevatorMotorCANID = 9;
        public static final int kFollowElevatorMotorCANID = 10;

        public static final double kElevatorSpraketDiameterInches = 1.63;
        
        public static final double kDt = 0.02;
        public static final double kMaxVelocity = Units.metersToInches(2.0);
        public static final double kMaxAcceleration = Units.metersToInches(1.5);
        public static final double kP = 0.15;/*1.3;*/
        public static final double kI = 1e-4;
        public static final double kD = 0.0;
        public static final double kS = 0.1;/*1.1;*/
        public static final double kG = 0.65;/*1.2;*/
        public static final double kV = 0;/*1.3;*/

        public static final TrapezoidProfile.Constraints elevConstraints =
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

        // public static final double kS = 0.5;
        // public static final double kG = 2.5;
        // public static final double kV = 1.8;
        // public static final double kA = 0.12;
        

        public static final double kElevatorMaxHeightInches = 47.5;
        public static final double kElevatorMaxSpeed = 1.0;

        // public static final double kMaxVelocityRaw = 1500;
        // public static final double kMaxAccelerationRaw = 400;

        public static final double kElevatorMotorGearRatio = 15.0;

        public static final double kElevatorForwardSoftLimit = kElevatorMaxHeightInches;
        public static final double kElevatorReverseSoftLimit = 0;

        public static final double kElevatorHeightDeadbandInches = 0.35;

        public static final double kElevatorPosition_L0 = 1.25;
        public static final double kElevatorPosition_L1 = 10;
        public static final double kElevatorPosition_L2 = 25;
        public static final double kElevatorPosition_L3 = 47.5;
    }

    public static final class ArmConstants{
        public static final int kArmMotorCANID = 13;
        public static final int kArmRollerMotorCANID = 14;

        public static final double kArmMotorGearRatio = 75;
        public static final double kArmRollerMotorGearRatio = 4;

        public static final double kS = 0.001;/*1.1;*/
        public static final double kG = 0.0025;/*1.2;*/
        public static final double kV = 0;/*1.3;*/


        public static final double kArmPositionDeadband = 01;
        public static final double kArmRollerSpeedDeadband = 100;

        public static final double kArmForwardSoftLimit = 195;
        public static final double kArmReverseSoftLimit = 10;

        public static final double kFullExtendPosition = 190;
        public static final double kStowPosition = 15;
        public static final double kPartialPosition = 30;
    }

    public static final class ScorerConstants{
        public static final int kScorerRightMotorCANID = 16;
        public static final int kScorerLeftMotorCANID = 15;

        public static final double kSensorProxDistanceMM = 8;
        public static final int kInitLaserCANID = 21;
        public static final int kEndLaserCANID = 22;

        public static final double kIntakeDuty = 0.6;

        public static final double kScorerMotorGearRatio = 5;

        public static final double kScorerSpeedDeadbandRPM = 120;
    }

    public static final class WinchConstants{
        public static final int kTrapCANID = 12;
        public static final int kWinchCANID = 11;

        public static final double kTopPosition = 210;
        public static final double kIdlePosition = -90; 

        public static final double kIdleSpeed = -0.25;
        public static final double kWinchSpeed = 0.90;

        public static final double trapOpenSpeed = - 0.3;
        public static final double trapCloseSpeed = 0.45;
    }
}
