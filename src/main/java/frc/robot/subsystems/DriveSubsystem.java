package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;

// declares to be a part of the subsystem framework
public class DriveSubsystem extends SubsystemBase {

    // instances of modules for the each of the wheel of the robot
    private final MAXSwereveModule kFLeft = new MAXSwereveModule(DriveConstants.kFrontLeftSteerCANID,
            DriveConstants.kFrontLeftDriveCANID, DriveConstants.kFrontLeftOffset);
    private final MAXSwereveModule kFRight = new MAXSwereveModule(DriveConstants.kFrontRightSteerCANID,
            DriveConstants.kFrontRightDriveCANID, DriveConstants.kFrontRightOffset);
    private final MAXSwereveModule kBLeft = new MAXSwereveModule(DriveConstants.kBackLeftSteerCANID,
            DriveConstants.kBackLeftDriveCANID, DriveConstants.kBackLeftOffset);
    private final MAXSwereveModule kBRight = new MAXSwereveModule(DriveConstants.kBackRightSteerCANID,
            DriveConstants.kBackRightDriveCANID, DriveConstants.kBackRightOffset);

    // instance of gyro for orientantion
    // parameter for the gyro specifies the port for connection
    private final AHRS gyro = new AHRS(NavXComType.kUSB1);

    private double fieldRelativeOffset = 0;

    
    // movement variables
    private double currentRotation = 0;
    private double currentDirection = 0;
    private double currentMagnitude = 0;

    // limits and constraints for acceleration to prevent sudden changes
    private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.magLimiterSlewRate);
    // limits the rotational change
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.rotationSlewRate);

    // instance to keep track of timestamp and utilize it as needed
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    // instance for tracing the robot's position(odometer)
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(-gyro.getAngle()), new SwerveModulePosition[] {
                    kFLeft.getPosition(), 
                    kFRight.getPosition(), 
                    kBLeft.getPosition(), 
                    kBRight.getPosition()
            });

    // adjusting the defaults for the robot's periodic framework
    @Override
    public void periodic() {
        // updates the odometry using the current values gyro and position
        odometry.update(
                Rotation2d.fromDegrees(-gyro.getAngle()),
                new SwerveModulePosition[] {
                        kFLeft.getPosition(),
                        kFRight.getPosition(),
                        kBLeft.getPosition(),
                        kBRight.getPosition()
                });
        SmartDashboard.putString("odometry", odometry.getPoseMeters().toString());
        SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    }

    // returns the current position of the robot as a 'Pose2d' object
    public Pose2d getP() {
        return odometry.getPoseMeters();
    }

    // resets the odometry by utilizing the current gyro angle and module positions
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()),
                new SwerveModulePosition[] { kFLeft.getPosition(),
                        kFRight.getPosition(),
                        kBLeft.getPosition(),
                        kBRight.getPosition() },
                pose);
    }

    // controls the robot's movement by setting desired velocity, which is
    // respective to field relative and also responsive to limits of the rate of
    // change
    public void drive(double x, double y, double rotation, boolean fieldrelative, boolean ratelimit) {
        // variables for the limited speeds
        double limitedx;
        double limitedy;

        // rate limiting function
        if (ratelimit) {
            // input variables for the direction and magnitude(input vectors)
            double inputdirection = Math.atan2(y, x);
            double inputmagnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            // variable for the change in direction
            double directionrate;

            // calculates the directionrate based on the magnitude
            if (currentMagnitude != 0.0) {
                // greater the magnitude, less the directionrate derived from the slewrate
                directionrate = Math.abs(DriveConstants.directionSlewRate / currentMagnitude);
            } else {
                // zero magnitude results in high directionrate
                directionrate = 500.0;
            }

            // current timestamp in seconds
            double currenttime = WPIUtilJNI.now() * 1e-6;
            // time elapsed since the last update
            double deltatime = currenttime - prevTime;
            // angle difference between the current direction and the input direction
            double anglediff = SwerveUtils.AngleDifference(inputdirection, currentDirection);

            // function for efficiency in rotation
            // if the current angle difference is small.....
            if (anglediff < 0.45 * Math.PI) {
                // updates the current direction
                currentDirection = SwerveUtils.StepTowardsCircular(currentDirection, inputdirection,
                        directionrate * deltatime);
                // updates magnitude smoothly towards the input values
                currentMagnitude = magLimiter.calculate(inputmagnitude);
            }

            // if the angle difference is large.....
            else if (anglediff > 0.85 * Math.PI) {

                // if magnitude is more that zero, reduces it to zero (stop)
                if (currentMagnitude > 1e-4) {
                    currentMagnitude = magLimiter.calculate(0.0);
                }

                // once the magnitude is zero.....
                else {
                    // flips the direction by PI radians(wraps)
                    currentDirection = SwerveUtils.WrapAngle(currentDirection + Math.PI);
                    // sets new magnitude
                    currentMagnitude = magLimiter.calculate(inputmagnitude);
                }
            }

            // for medium differences.....
            else {
                // updates the current direction smoothly
                currentDirection = SwerveUtils.StepTowardsCircular(currentDirection, inputdirection,
                        directionrate * deltatime);
                // reduces the magnitude to zero
                currentMagnitude = magLimiter.calculate(0.0);
            }
            // updates the previous time to current time and cumulate the timestamp
            prevTime = currenttime;

            // calculates the limited version of speeds based on the current magnitude and
            // direction
            limitedx = currentMagnitude * Math.cos(currentDirection);
            limitedy = currentMagnitude * Math.sin(currentDirection);
            // limits the roational speed using rot slew rate limiter
            currentRotation = rotLimiter.calculate(rotation);
        }

        // if the rates are not being limited, it uses the raw input directly
        else {
            limitedx = x;
            limitedy = y;
            currentRotation = rotation;
        }

        // scales and finalizes the limited speeds with the maximum allowed speeds
        double finalx = limitedx * DriveConstants.kMaxSpeedMetersPerSec;
        double finaly = limitedy * DriveConstants.kMaxSpeedMetersPerSec;
        double finalrot = currentRotation * DriveConstants.kMaxAngSpeedRadiansPerSec;

        // converts and translates the final speeds to swerve module states referring
        // back to drive system kinematics
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                // depending on field relative while using ternary operator(small conditional),
                // speeds are converted to robot-relative speeds
                fieldrelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(finalx, finaly, finalrot,
                                Rotation2d.fromDegrees(-gyro.getAngle()).plus(Rotation2d.fromDegrees(fieldRelativeOffset)))
                        : new ChassisSpeeds(finalx, finaly, finalrot));

        // checks and makes sure none of the module states exceeds the max allowed
        // speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSec);

        // sets the actual converted swerve module states
        kFLeft.setState(moduleStates[0]);
        kFRight.setState(moduleStates[1]);
        kBLeft.setState(moduleStates[2]);
        kBRight.setState(moduleStates[3]);
    }

    // sets the wheels into X formation to prevent movement(essentially brake)
    public void setX() {
        kFLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        kFRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        kBLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        kBRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, AutoConstants.kMaxSpeedMetersPerSecondStandard);
        kFLeft.setState(desiredStates[0]);
        kFRight.setState(desiredStates[1]);
        kBLeft.setState(desiredStates[2]);
        kBRight.setState(desiredStates[3]);
    }

    // resets the drive encoders
    public void resetEncoder() {
        kFLeft.resetEncoder();
        kFRight.resetEncoder();
        kBLeft.resetEncoder();
        kBRight.resetEncoder();
    }

    // resets the gyro heading to zero
    public void zeroHeading() {
        gyro.reset();
        fieldRelativeOffset = 0;
    }

    // returns robot's heading in degrees (reading from gyro)
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle(/*IMUAxis.kZ */)).getDegrees();
    }

    public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(gyro.getAngle(/*IMUAxis.kZ*/));
    }

    public double getTurnState() {
        return gyro.getRate();
    }

    public void setFieldRelativeOffset(double offset) {
        this.fieldRelativeOffset = offset;
    }
}