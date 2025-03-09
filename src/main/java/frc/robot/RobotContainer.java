// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DefaultDriveCommand;
// import frc.robot.commands.LedCycleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ScorerSubsystem;
import frc.robot.subsystems.WinchSubsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


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

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /*---------------------------------------------Only For Center Auton----------------------------------------------- */

    new EventTrigger("elevatorUpL1").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1), elevator));

    new EventTrigger("armOut").onTrue(
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(() -> arm.setArmRoller(-.4), arm));

    new EventTrigger("armDown+eject").onTrue(
      new InstantCommand(() -> arm.setArmPosition(15), arm).andThen(() -> arm.setArmRoller(.3))
    );

    new EventTrigger("elevatorUpL2").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2))
    );

    NamedCommands.registerCommand("ejectCoral", new InstantCommand(() -> scorer.ejectElevated(), scorer)
      .andThen(new WaitUntilCommand(scorer::notHasCoral)).andThen(new InstantCommand(() -> scorer.stop(), scorer)));
    
    new EventTrigger("elevatoDownL0").onTrue(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator));

    new EventTrigger("elevatorUpL2+armOut").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2), elevator),
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(() -> arm.setArmRoller(-.4), arm)));

    new EventTrigger("elevatorDownL0+armDown").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0), elevator),
      new InstantCommand(() -> arm.setArmPosition(15), arm).andThen(() -> arm.setArmRoller(-.4), arm)));

    NamedCommands.registerCommand("ejectAlgae", new InstantCommand(() -> arm.setArmRoller(.3), arm).
      andThen(new WaitCommand(1)).andThen(new InstantCommand(() -> arm.setArmRoller(0), arm)));
    

    /*---------------------------------------------Only For Side Auton----------------------------------------------- */
    new EventTrigger("initiateIntake").onTrue(new CoralIntakeCommand(scorer));
    
    new EventTrigger("elevatorDownL1").onTrue(
      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L1), elevator));

    

    new EventTrigger("armDown").onTrue(
      new InstantCommand(() -> arm.setArmPosition(25), arm).andThen(() -> arm.setArmRoller(.3), arm));

    new EventTrigger("armUpL2").onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> elevator.setPosition(23)), 
      new InstantCommand(() -> arm.setArmPosition(90), arm).andThen(() -> arm.setArmRoller(-.4), arm))
    );
    
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("Center");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  
  private void configureBindings() {
    // elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));
    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));
    // led.setDefaultCommand(new LedCycleCommand(led, scorer));

    driverControllerCommand.a().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.y().whileTrue(new RunCommand(() -> robotDrive.zeroHeading()));

    driverControllerCommand.leftBumper().whileTrue(new StartEndCommand(() -> winch.openTrap(), () -> winch.stopTrap()));
    driverControllerCommand.rightBumper().whileTrue(new StartEndCommand(() -> winch.closeTrap(), () -> winch.stopTrap()));

    // driverControllerCommand.a().whileTrue(robotDrive.sysIdQuasistatic(Direction.kForward));
    // driverControllerCommand.y().whileTrue(robotDrive.sysIdQuasistatic(Direction.kBackward)));

    // driverControllerCommand.leftBumper().whileTrue(robotDrive.sysIdDynamic(Direction.kForward));
    // driverControllerCommand.rightBumper().whileTrue(robotDrive.sysIdDynamic(Direction.kBackward));


    // driverControllerCommand.a().whileTrue(new InstantCommand(() -> winch.openTrap()));
    // driverControllerCommand.b().whileTrue(new InstantCommand(() -> winch.closeTrap()));

    coPilotControllerCommand.y().whileTrue(new InstantCommand(() -> elevator.incremPos(), elevator));
    coPilotControllerCommand.b().onTrue(new InstantCommand(() -> arm.incremPos(), arm));
    coPilotControllerCommand.x().onTrue(new InstantCommand(() -> arm.decremPos(), arm));
    coPilotControllerCommand.a().whileTrue(new InstantCommand(() -> elevator.decremPos(), elevator));

    new JoystickButton(copilotController, Button.kLeftBumper.value).whileTrue(new StartEndCommand(() -> scorer.ejectBottomLeft(), 
      () -> scorer.stopScorer()));
    new JoystickButton(copilotController, Button.kRightBumper.value).whileTrue(new StartEndCommand(() -> scorer.ejectBottomRight(), 
      () -> scorer.stopScorer()));

    new Trigger(this::leftTrigger).whileTrue(new CoralIntakeCommand(scorer));
    new Trigger(this::rightTrigger).whileTrue(new StartEndCommand(() -> scorer.ejectElevated(), () -> scorer.stopScorer()));
    new Trigger(this::rightTrigger).onTrue(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new StartEndCommand(() -> scorer.ejectElevated(), () -> scorer.stopScorer(), scorer)
                    .withTimeout(1.0), 
                new InstantCommand(() -> LedSubsystem.setAllianceSolid())
            ),
            new InstantCommand(() -> LedSubsystem.setAllianceSolid()),
            scorer::hasCoral
        )
    );

    new Trigger(this::R1Left).whileTrue(new StartEndCommand(() -> arm.setArmRoller(-0.40), () -> arm.setArmRoller(0)));
    new Trigger(this::R1Right).whileTrue(new StartEndCommand(() -> arm.setArmRoller(0.3), () -> arm.setArmRoller(0)));

    new Trigger(this::R1Up).whileTrue(new InstantCommand(() -> arm.setArmPosition(90), arm));
    new Trigger(this::R1Down).whileTrue(new InstantCommand(() -> arm.setArmPosition(15), arm));

    new Trigger(elevator::atDangerHeight)
            .onTrue(new InstantCommand(() -> LedSubsystem.scrollMsg()))
            .onFalse(new InstantCommand(() -> LedSubsystem.setAllianceSolid()));

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
  private boolean R1Left(){
    return copilotController.getRawAxis(4) < -0.75;
  }
  private boolean R1Right(){
    return copilotController.getRawAxis(4) > 0.75;
  }
  // private boolean L1Down() {
  //   return copilotController.getRawAxis(1) > 0.75;
  // }
  // private boolean L1Up() {
  //   return copilotController.getRawAxis(1) < -0.75;
  // }

  public Command getAutonomousCommand() {

    robotDrive.zeroHeading();
    // robotDrive.setFieldRelativeOffset(180);
    return autoChooser.getSelected();
    // robotDrive.resetOdometry(new Pose2d(new Translation2d(6.7631, 4.195), new Rotation2d(180)));
    // return exampleCommand().andThen(new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2)))
    // .andThen(new WaitCommand(3)).andThen(() -> scorer.ejectElevated(), scorer); // might be brake as true.
    
    // return new ParallelCommandGroup(moveForwardCommand(true), 
    // new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2)))
    // .andThen(new WaitUntilCommand(elevator::atHeight))
    // .andThen(new WaitCommand(1.5))
    // .andThen(() -> scorer.ejectElevated(), scorer)
    // .andThen(new WaitUntilCommand(scorer::notHasCoral));
    // .andThen(centerReef_Algae1()).andThen(() -> {
    //   scorer.stop();
    //   arm.setArmPosition(90);
    //   arm.setArmRoller(-.4);
    // })
    // .andThen(new WaitCommand(.5));

  }

  // private Command exampleCommand(){
  //       // Create config for trajectory
  //   TrajectoryConfig config = new TrajectoryConfig(
  //       AutoConstants.kMaxSpeedMetersPerSecondSlow/2,
  //       AutoConstants.kMaxAccelerationMetersPerSecondSquaredSlow/2)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DriveConstants.kDriveKinematics);

  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(1, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(new Translation2d(1.5, 0), new Translation2d(1.75, 0)),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(2, 0, new Rotation2d(1)),
  //       config);

  //   var thetaController = new ProfiledPIDController(
  //       AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //       exampleTrajectory,
  //       robotDrive::getP, // Functional interface to feed supplier
  //       DriveConstants.kDriveKinematics,

  //       // Position controllers
  //       new PIDController(AutoConstants.kPXController*1, 0, 0),
  //       new PIDController(AutoConstants.kPYController*1, 0, 0),
  //       thetaController,
  //       robotDrive::setModuleStates,
  //       robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   robotDrive.resetOdometry(new Pose2d(1, 0, new Rotation2d(0)));

  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand/*.andThen(() -> robotDrive.drive(0, 0, 0, false,false))*/;

  // }

  public void setSmartDashboard(){
    // SendableChooser<AutoType> autoType = new SendableChooser<AutoType>();
    // SendableChooser<AutoPos> autoPos = new SendableChooser<AutoPos>();
    // SendableChooser<AutoAngle> autoAngle = new SendableChooser<AutoAngle>();

    // autoType.addOption("One Piece", AutoType.One_Piece);
    // autoType.addOption("Two Piece", AutoType.Two_Piece);
    // autoType.addOption("Three Piece", AutoType.Three_Piece);
    // autoType.addOption("Four Piece", AutoType.Four_Piece);
    // autoType.setDefaultOption("Four Piece", AutoType.Three_Piece);
    // SmartDashboard.putData("Auto Type", autoType);

    // autoPos.addOption("Left", AutoPos.Left);
    // autoPos.addOption("Center", AutoPos.Center);
    // autoPos.addOption("Right", AutoPos.Right);
    // autoPos.setDefaultOption("Left", AutoPos.Left);
    // SmartDashboard.putData("Auto Pos", autoPos);

    // autoAngle.addOption("Left", AutoAngle.Left);
    // autoAngle.addOption("Right", AutoAngle.Right);
    // autoAngle.setDefaultOption("Right", AutoAngle.Right);
    // SmartDashboard.putData("Auto Angle", autoAngle);
  }

//   private Command center_Zero_Algae(){
//     return new ParallelCommandGroup(moveForwardCommand(true), 
//     new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L2), elevator))
//     .andThen(new WaitUntilCommand(elevator::atHeight))
//     .andThen(() -> scorer.ejectElevated(), scorer)
//     .andThen(new WaitUntilCommand(scorer::notHasCoral))
//     .andThen(new ParallelCommandGroup(centerReef_Algae1(),
//       new InstantCommand(() -> scorer.stop(), scorer),
//       new InstantCommand(() -> arm.setArmPosition(90), arm),
//       new InstantCommand(() -> arm.setArmRoller(-.4), arm)
//     ))
//     .andThen(new WaitCommand(.5))
//     .andThen(new ParallelCommandGroup(centerReef_Algae2(),
//         new InstantCommand(() -> arm.setArmRoller(.3), arm),
//         new InstantCommand(() -> arm.setArmPosition(30), arm)
//     ))
//       .andThen(new ParallelCommandGroup(centerReef_Algae2_half(), 
//         new InstantCommand(() -> arm.setArmRoller(-.4), arm),
//         new InstantCommand(() -> arm.setArmPosition(90), arm)
//       ))
//     .andThen(new WaitCommand(.5))
//     .andThen(new ParallelCommandGroup(centerReef_Algae3(),
//         new InstantCommand(() -> arm.setArmRoller(.3), arm),
//         new InstantCommand(() -> arm.setArmPosition(30), arm)
//     ))
//     .andThen(new ParallelCommandGroup(centerReef_Algae3_half(), 
//     new InstantCommand(() -> arm.setArmRoller(-.4), arm),
//     new InstantCommand(() -> arm.setArmPosition(90), arm)
//   ));
// }

//   private Command left_Three_Coral(){
//     return leftReef_Coral1()
//     .andThen(new ParallelCommandGroup(new InstantCommand(
//       () -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L3)), leftReef_Coral2()))
//     .andThen(new WaitUntilCommand(elevator::atHeight))
//     .andThen(() -> new RunCommand(() -> scorer.ejectElevated(), scorer))
//     .andThen(new ParallelRaceGroup(new WaitUntilCommand(() -> !scorer.hasCoral()), new WaitCommand(0.35)))
//     .andThen(new ParallelCommandGroup(leftReef_Coral3(), 
//       new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L0))))
//     .andThen(new ParallelCommandGroup(leftReef_Coral4(), new CoralIntakeCommand(scorer)))
//     .andThen(new ParallelRaceGroup(new WaitUntilCommand(scorer::holdingCoral), new WaitCommand(1.5)))
//     .andThen(leftReef_Coral5())
//     .andThen(new ParallelCommandGroup(leftReef_Coral6(),
//      new InstantCommand(() -> elevator.setPosition(ElevatorConstants.kElevatorPosition_L3))))
//     .andThen(() -> new RunCommand(() -> scorer.ejectElevated(), scorer));
//   }

//   public enum AutoPos{
//     Left, Center, Right
//   }

//   public enum AutoType {
//     One_Piece,
//     Two_Piece,
//     Three_Piece,
//     Four_Piece
//   }

//   public enum AutoPiece {
//     Coral,
//     Algae
//   }

//   public enum AutoAngle {
//       Right,
//       Left
//   }

//   public void setFieldRelativeOffset(double offset) {
//     robotDrive.setFieldRelativeOffset(offset);
//   }

//   public ProfiledPIDController getThetaController() {
//     var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController * 1.25, 0, 0, AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     return thetaController;
//   }

//   private Command swerveCommand(Trajectory traj, boolean brake){
//     SwerveControllerCommand command = new SwerveControllerCommand(
//       traj,
//       robotDrive::getP, 
//       DriveConstants.kDriveKinematics, 
//       new PIDController(1, 0, 0), 
//       new PIDController(1, 0, 0),
//       getThetaController(),
//       robotDrive::setModuleStates,
//       robotDrive);
//     if (brake) {
//       return command.andThen(() -> robotDrive.drive(0, 0, 0, true, true));
//     }
//     else{
//       return command;
//     }
//   }

//   private Command moveForwardCommand(boolean brake){
//       Trajectory moveForwardTraj = TrajectoryGenerator.generateTrajectory(
//           new Pose2d(new Translation2d(6.7631, 4.195), Rotation2d.fromDegrees(180)),
//           List.of(),
//           new Pose2d(new Translation2d(6.2185, 4.195), Rotation2d.fromDegrees(180)),
//           AutoConstants.kTrajConfigSlow);

//           SwerveControllerCommand command = new SwerveControllerCommand(
//             moveForwardTraj,
//             robotDrive::getP, 
//             DriveConstants.kDriveKinematics, 
//             new PIDController(1, 0, 0), 
//             new PIDController(1, 0, 0),
//             getThetaController(),
//             () -> Rotation2d.fromDegrees(180),
//             robotDrive::setModuleStates,
//             robotDrive);

//           robotDrive.resetOdometry(new Pose2d(new Translation2d(6.7631, 4.195), Rotation2d.fromDegrees(180)));
//         if (brake) {
//           return command.andThen(() -> robotDrive.drive(0, 0, 0, true, true));
//         }
//         else{
//           return command;
//         }
//   }

//   private Command centerReef_Algae1(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(6.2185, 4.195), new Rotation2d(0)),
//       List.of(new Translation2d(6.67, 4.092)),
//       new Pose2d(new Translation2d(6.568, 4.023), new Rotation2d(0)), 
//       AutoConstants.kTrajConfigSlow);

//       return swerveCommand(cenTrajectory, false);
//   }

//   private Command centerReef_Algae2(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(6.568, 4.023), Rotation2d.fromDegrees(0)),
//       List.of(
//         /*new Pose2d(*/new Translation2d(6.3422, 2.707), /*new Rotation2d.fromDegrees(155)),*/
//         /*new Pose2d(*/new Translation2d(6.1773, 2.3361)/* , new Rotation2d.fromDegrees(140))*/
//         ),
//         new Pose2d(new Translation2d(5.781, 1.8467), Rotation2d.fromDegrees(Math.PI/3)),
//       AutoConstants.kTrajConfigStandard);

//       return swerveCommand(cenTrajectory, false);
//   }

//   private Command centerReef_Algae2_half(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(5.781, 1.8467), Rotation2d.fromDegrees(120)),
//       List.of(),
//         new Pose2d(new Translation2d(5.573, 2.235), Rotation2d.fromDegrees(120)),
//       AutoConstants.kTrajConfigSlow);

//       return swerveCommand(cenTrajectory, false);
//   }

//   private Command centerReef_Algae3(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(5.573, 2.235), Rotation2d.fromDegrees(120)),
//       List.of(
//       new Translation2d(6.28, 3.314),
//       new Translation2d(6.7631, 4.195),
//       new Translation2d(6.218, 4.623)
//       ),
//       new Pose2d(new Translation2d(5.6421, 6.023), Rotation2d.fromDegrees(250)),
//       AutoConstants.kTrajConfigStandard);

//       return swerveCommand(cenTrajectory, false);
//   }

//   private Command centerReef_Algae3_half(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(5.6421, 6.023), Rotation2d.fromDegrees(250)),
//       List.of(),
//       new Pose2d(new Translation2d(5.52, 5.805), Rotation2d.fromDegrees(250)),
//       AutoConstants.kTrajConfigSlow);

//       return swerveCommand(cenTrajectory, false);
//   }

//   private Command leftReef_Coral1(){ // rush
//     Trajectory traj = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(7.593, 6.753), Rotation2d.fromDegrees(225)),
//       List.of(
//       new Translation2d(6.202, 5.501)),
//       new Pose2d(new Translation2d(5.689, 5.362), Rotation2d.fromDegrees(250)),
//       AutoConstants.kTrajConfigFast);

//       robotDrive.resetOdometry(traj.getInitialPose());

//       return swerveCommand(traj, true);
//   }

//   private Command leftReef_Coral2(){
//       Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(5.689, 5.362), Rotation2d.fromDegrees(250)),
//       List.of(),
//       new Pose2d(new Translation2d(5.4534, 5.442), Rotation2d.fromDegrees(250)), 
//       AutoConstants.kTrajConfigSlow);

//       return swerveCommand(cenTrajectory, true);
//   }

//   private Command leftReef_Coral3(){
//     Trajectory traj = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(5.4534, 5.442), Rotation2d.fromDegrees(250)),
//       List.of(),
//       new Pose2d(new Translation2d(4.977, 5.806), Rotation2d.fromDegrees(260)), 
//       AutoConstants.kTrajConfigSlow);

//       return swerveCommand(traj, false);
//   }

//   private Command leftReef_Coral4(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(4.977, 5.806), Rotation2d.fromDegrees(260)),
//       List.of(
//       new Translation2d(4.12, 6.076),
//       new Translation2d(2.8563, 6.4375)
//       ),
//       new Pose2d(new Translation2d(1.371, 6.753), Rotation2d.fromDegrees(310)),
//       AutoConstants.kTrajConfigFast);

//       return swerveCommand(cenTrajectory, true);
//   }

//   private Command leftReef_Coral5(){
//     Trajectory traj = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(1.371, 6.753), Rotation2d.fromDegrees(310)),
//       List.of(
//       new Translation2d(2.3297, 6.6179)),
//       new Pose2d(new Translation2d(3.278, 5.625), Rotation2d.fromDegrees(310)),
//       AutoConstants.kTrajConfigStandardReverse);

//       return swerveCommand(traj, false);
//   }

//   private Command leftReef_Coral6(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(3.278, 5.625), Rotation2d.fromDegrees(310)),
//       List.of(),
//       new Pose2d(new Translation2d(3.7873, 5.5774), Rotation2d.fromDegrees(310)),
//       AutoConstants.kTrajConfigSlowReverse);

//       return swerveCommand(cenTrajectory, true);
//   }

//   private Command leftReef_Coral7(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(3.7873, 5.5774), Rotation2d.fromDegrees(310)),
//       List.of(
//       new Translation2d(2.616, 6.362)),
//       new Pose2d(new Translation2d(1.371, 6.753), Rotation2d.fromDegrees(300)),
//       AutoConstants.kTrajConfigStandard);

//       return swerveCommand(cenTrajectory, true);
//   }

//   private Command leftReef_Coral8(){
//     Trajectory traj = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(1.371, 6.753), Rotation2d.fromDegrees(310)),
//       List.of(
//       new Translation2d(2.3297, 6.6179)
//       ),
//       new Pose2d(new Translation2d(3.5633, 5.715), Rotation2d.fromDegrees(310)),
//       AutoConstants.kTrajConfigStandardReverse);

//       return swerveCommand(traj, false);
//   }

//   private Command leftReef_Coral9(){
//     Trajectory cenTrajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(new Translation2d(3.5633, 5.715), Rotation2d.fromDegrees(310)),
//       List.of(),
//       new Pose2d(new Translation2d(3.4979, 5.4226), Rotation2d.fromDegrees(310)), 
//       AutoConstants.kTrajConfigSlowReverse);

//       return swerveCommand(cenTrajectory, true);
//   }
  

}
