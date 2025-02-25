// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScorerSubsystem;
import frc.robot.subsystems.WinchSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final WinchSubsystem winch = new WinchSubsystem();
  private final ScorerSubsystem scorer = new ScorerSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverControllerCommand =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController coPilotControllerCommand =
      new CommandXboxController(OIConstants.kCoPilotControllerPort);

  private final XboxController copilotController = new XboxController(OIConstants.kCoPilotControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  
  private void configureBindings() {
    // elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));
    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));

    driverControllerCommand.x().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.y().whileTrue(new RunCommand(() -> robotDrive.zeroHeading()));

    coPilotControllerCommand.a().whileTrue(new InstantCommand(() -> arm.setArmPosition(200), arm));
    coPilotControllerCommand.b().whileTrue(new StartEndCommand(() -> arm.setArmRoller(0.3), () -> arm.setArmRoller(0)));
    coPilotControllerCommand.x().whileTrue(new StartEndCommand(() -> arm.setArmRoller(-0.25), () -> arm.setArmRoller(0)));
    coPilotControllerCommand.y().whileTrue(new InstantCommand(() -> arm.setArmPosition(110), arm));

    new JoystickButton(copilotController, Button.kLeftBumper.value).whileTrue(new StartEndCommand(() -> scorer.ejectBottomLeft(), 
      () -> scorer.stopScorer()));
    new JoystickButton(copilotController, Button.kRightBumper.value).whileTrue(new StartEndCommand(() -> scorer.ejectBottomRight(), 
      () -> scorer.stopScorer()));
    // new Trigger(this::leftTrigger).whileTrue(new RunCommand(() -> scorer.intake(), scorer));
    new Trigger(this::rightTrigger).whileTrue(new StartEndCommand(() -> scorer.ejectElevated(), () -> scorer.stopScorer()));

    coPilotControllerCommand.povUp().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L3), elevator));
    coPilotControllerCommand.povDown().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator));
    coPilotControllerCommand.povRight().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1), elevator));
    coPilotControllerCommand.povLeft().onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2), elevator));

  }

  private boolean leftTrigger() {
    return copilotController.getRawAxis(2) > 0.75;
  }
  private boolean rightTrigger() {
    return copilotController.getRawAxis(3) > 0.75;
  }
  private boolean R1Down() {
    return copilotController.getRawAxis(5) > 0.75;
  }
  private boolean R1Up() {
    return copilotController.getRawAxis(5) < -0.75;
  }
  private boolean L1Down() {
    return copilotController.getRawAxis(1) > 0.75;
  }
  private boolean L1Up() {
    return copilotController.getRawAxis(1) < -0.75;
  }

  public Command getAutonomousCommand() {
    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecondStandard,
        AutoConstants.kMaxAccelerationMetersPerSecondSquaredStandard)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        robotDrive::getP, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        robotDrive::setModuleStates,
        robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false,false));
  }

  public void setSmartDashboard(){
    SendableChooser<AutoType> autoType = new SendableChooser<AutoType>();
    autoType.addOption("One Piece", AutoType.One_Piece);
    autoType.addOption("Two Piece", AutoType.Two_Piece_Right);
    autoType.addOption("Two Piece", AutoType.Two_Piece_Left);
    autoType.addOption("Three Piece Right", AutoType.Three_Piece_Right);
    autoType.addOption("Four Piece Left", AutoType.Four_Piece_Left);
    autoType.addOption("Four Piece Right", AutoType.Four_Piece_Right);
    autoType.setDefaultOption("Three Piece Left", AutoType.Three_Piece_Left);
    SmartDashboard.putData("Auto Type", autoType);
  }

  public enum AutoType {
    One_Piece,
    Two_Piece_Left,
    Two_Piece_Right,
    Three_Piece_Left,
    Three_Piece_Right,
    Four_Piece_Left,
    Four_Piece_Right
  }

  public enum AutoPiece {
    Coral,
    Algae
  }

  public enum AutoAngle {
      Right,
      Left
  }

  public void setFieldRelativeOffset(double offset) {
    robotDrive.setFieldRelativeOffset(offset);
  }

  public ProfiledPIDController getThetaController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(1.25, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return thetaController;
  }

  private Command swerveCommand(Trajectory traj){
    SwerveControllerCommand command = new SwerveControllerCommand(
      traj,
      robotDrive::getP, 
      DriveConstants.kDriveKinematics, 
      new PIDController(1, 0, 0), 
      new PIDController(1, 0, 0),
      getThetaController(),
      robotDrive::setModuleStates,
      robotDrive);
    return command.andThen(() -> robotDrive.drive(0, 0, 0, true, true));
  }

  private Command moveForwardCommand(){
      Trajectory moveForwardTraj = TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(7.1927, 4.195), Rotation2d.fromDegrees(180)),
          List.of(),
          new Pose2d(new Translation2d(5.7133, 4.195), Rotation2d.fromDegrees(180)),
          AutoConstants.kTrajConfigSlow);
      SwerveControllerCommand moveToReefCommand = new SwerveControllerCommand(
          moveForwardTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command moveToCoralStationCommand_1(){
    Trajectory moveToCoralStationTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(5.7133, 4.195), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(5.7133, 4.795), Rotation2d.fromDegrees(200)),
        new Pose2d(new Translation2d(2.265, 6.205), Rotation2d.fromDegrees(300)),
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigStandard);
    SwerveControllerCommand moveToCoralStationCommand = new SwerveControllerCommand(
          moveToCoralStationTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveToCoralStationCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command rushTowardsReefCommand_1(){
    Trajectory rushTowardsReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.025, 6.235), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.405, 5.375), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigFast);
    SwerveControllerCommand rushTowardsReefCommand = new SwerveControllerCommand(
          rushTowardsReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return rushTowardsReefCommand;
  }

  private Command moveToReefCommand_2(){
    Trajectory moveToReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.405, 5.375), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.655, 4.948), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigSlow);
    SwerveControllerCommand moveToReefCommand = new SwerveControllerCommand(
          moveToReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command moveToCoralStationCommand_2(){
    Trajectory moveBackToCoralStationTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.655, 4.948), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.246, 6.386), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigStandard);
    SwerveControllerCommand moveBackToCoralStationCommand = new SwerveControllerCommand(
          moveBackToCoralStationTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveBackToCoralStationCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command rushTowardsReefCommand_2(){
    Trajectory againRushTowardsReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.195, 6.235), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.589, 5.375), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigFast);
    SwerveControllerCommand againRushTowardsReefCommand = new SwerveControllerCommand(
          againRushTowardsReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againRushTowardsReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command moveToReefCommand_3(){
    Trajectory againMoveToReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.589, 5.375), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.951, 5.097), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigSlow);
    SwerveControllerCommand againMoveToReefCommand = new SwerveControllerCommand(
          againMoveToReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againMoveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command moveToCoralStationCommand_3(){
    Trajectory againMoveToCoralStationTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.951, 5.097), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.051, 6.266), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigStandard);
    SwerveControllerCommand againMoveToCoralStationCommand = new SwerveControllerCommand(
          againMoveToCoralStationTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againMoveToCoralStationCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command rushTowardsReefCommand_3(){
    Trajectory againRushBackTowardsReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.170, 6.386), Rotation2d.fromDegrees(335)),
        new Pose2d(new Translation2d(2.93, 4.324), Rotation2d.fromDegrees(0))),
      AutoConstants.kTrajConfigFast);
    SwerveControllerCommand againRushBackTowardsReefCommand = new SwerveControllerCommand(
          againRushBackTowardsReefTraj,
          robotDrive::getP,
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againRushBackTowardsReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command moveTowardsReefCommand_4(){
    Trajectory againMoveBackToReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(2.93, 4.324), Rotation2d.fromDegrees(0)),
        new Pose2d(new Translation2d(3.232, 4.182), Rotation2d.fromDegrees(0))),
      AutoConstants.kTrajConfigSlow);
    SwerveControllerCommand againMoveToReefCommand = new SwerveControllerCommand(
          againMoveBackToReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againMoveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

}
