// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

//Constants Imports 
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AllForNaught;
//import frc.robot.Trajectories;

//Command Imports
import frc.robot.commands.DriveToPointCmd;
import frc.robot.commands.RumbleCmd;
//import frc.robot.commands.LiftAutoCmd;
//import frc.robot.commands.ResetLiftBoundsCmd;
import frc.robot.commands.SwerveControllerCmd;
import frc.robot.commands.OTFTrajectoryFactory;
import frc.robot.commands.ShooterTriggerCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SullyCmd;
import frc.robot.commands.DropperAutoCmd;
import frc.robot.commands.DropperTriggerCmd;

//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.DropperSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import edu.wpi.first.wpilibj.util.Color;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ManipulatorSubsystem robotManipulator = new ManipulatorSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  private final DropperSubsystem dropperSubsystem = new DropperSubsystem();



  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);

  //COMMENT OUT THE CONTROLLER BELOW - ONLY USE WHEN NEED PS4 CONTROLLER
  //PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  //SendableChooser<Command> lightingChooser = new SendableChooser<>();
  
  PathConstraints standardAutoPathConstraints = new PathConstraints(AutoConstants.kAutoMaxSpeedMetersPerSecond, AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared);
  
//PathPlanner is for noobs
/* 
  //PathPlannerTrajectory defaultTrajectory = PathPlanner.loadPath("Default", standardAutoPathConstraints);
  PathPlannerTrajectory chargePlateLeftTrajectory = PathPlanner.loadPath("Charge Plate Run-up Left", standardAutoPathConstraints);
  PathPlannerTrajectory blueTopScoreTrajectory = PathPlanner.loadPath("Scoring Path Blue Top", standardAutoPathConstraints);
  PathPlannerTrajectory chargeRightTrajectory = PathPlanner.loadPath("Charge Plate Run-up Right", standardAutoPathConstraints);
*/
  PathPlannerTrajectory testForwardDrive = PathPlanner.loadPath("Test Forward Drive", standardAutoPathConstraints);
  PathPlannerTrajectory blueTopReturnCargo = PathPlanner.loadPath("Blue Top Return Cargo", standardAutoPathConstraints);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
  
    // Configure default commands

     
    robotDrive.setDefaultCommand(
        new SwerveControllerCmd(
            robotDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> false, // field oriented
            () -> false));

    shooterSubsystem.setDefaultCommand(
      new ShooterTriggerCmd(
        shooterSubsystem,
        () -> operatorController.getRightBumper(),
        () -> operatorController.getLeftBumper(),
        () -> operatorController.getAButton(),
        () -> operatorController.getYButton()));

    robotManipulator.setDefaultCommand(new IntakeCmd(robotManipulator, () -> 0.0, 0));

    //dropperSubsystem.setDefaultCommand(new DropperTriggerCmd(dropperSubsystem, 0));

    //lightingSubsystem.setDefaultCommand(lightingSubsystem.splitColor(Color.kAquamarine, Color.kDarkCyan));





    //**Load in paths from Trajectories as drive commands using the AutoCommandFactory**


   
    //VOID AUTO
    SwerveControllerCmd blank = new SwerveControllerCmd(
      robotDrive,
      () -> 0.0,
      () -> 0.0,
      () -> 0.0,
      () -> true,
      () -> false);

    //BACKUP AUTO
    SequentialCommandGroup goStraight = robotDrive.AutoCommandFactory(Trajectories.goStraight);
  
    //***TEST DROPPER AUTO
    SequentialCommandGroup driveAndDrop = new SequentialCommandGroup(robotDrive.AutoCommandFactory(Trajectories.goStraight),
    new DropperAutoCmd(dropperSubsystem));


    //ADD AUTONOMOUS COMMANDS TO SHUFFLEBOARD
    autoChooser.addOption("Void", blank);
    autoChooser.addOption("Go Straight Auto", goStraight);
    autoChooser.addOption("Drive and Drop Auto", driveAndDrop);




    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //Intake ball
    new JoystickButton(operatorController, Buttons.X).whileTrue(new IntakeCmd(robotManipulator, 
      () -> ManipulatorConstants.kIntakeMotorSpeed, 1));

    //Outake ball
    new JoystickButton(operatorController, Buttons.B).whileTrue(new IntakeCmd(robotManipulator, 
      () -> ManipulatorConstants.kIntakeMotorSpeed, -1));

    //Toggle intake
    new POVButton(operatorController, Buttons.DOWN_ARR).toggleOnTrue(new SullyCmd(pneumaticsSubsystem));

    //Down Bunny dropper
    new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new DropperTriggerCmd(dropperSubsystem, -1));

    //Up Bunney dropper
    new POVButton(driverController, Buttons.UP_ARR).whileTrue(new DropperTriggerCmd(dropperSubsystem, 1));

    //Slow intake
    new POVButton(operatorController, Buttons.RIGHT_ARR).whileTrue(new IntakeCmd(robotManipulator, () -> (0.25), -1));

    //Slow outake
    new POVButton(operatorController, Buttons.LEFT_ARR).whileTrue(new IntakeCmd(robotManipulator, () -> (0.25), 1));

    //Intake and shoot
    new JoystickButton(operatorController, Buttons.Maria).whileTrue(new ParallelCommandGroup(
      new IntakeCmd(robotManipulator, () -> ManipulatorConstants.kIntakeMotorSpeed, 1), 
      new ShooterTriggerCmd(
        shooterSubsystem,
        () -> false,
        () -> true,
        () -> false,
        () -> false)));

    //Outake and outshoot
    new JoystickButton(operatorController, Buttons.Menu).whileTrue(new ParallelCommandGroup(
      new IntakeCmd(robotManipulator, () -> ManipulatorConstants.kIntakeMotorSpeed, -1), 
      new ShooterTriggerCmd(
        shooterSubsystem,
        () -> true,
        () -> false,
        () -> false,
        () -> false)));

    //Rumble controllers
    new JoystickButton(driverController, Buttons.LB).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.L3).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.R3).whileTrue(new RumbleCmd(driverController, 2, 1.00));



    //Zero Heading
    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));

    //Slow drive with d-pad
    // new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true,  () -> false));
    // new POVButton(driverController, Buttons.UP_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true,  () -> false));
    // new POVButton(driverController, Buttons.RIGHT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true,  () -> false));
    // new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true,  () -> false));







    //Rotate robot to pick up cube
    new JoystickButton(driverController, Buttons.B).whileTrue(new SwerveControllerCmd(robotDrive, () -> (0.30), () -> (0.0), 
    () -> (-MathMethods.speedMax2(0.04*limelightSubsystem.getTargetOffsetX(), 0.2, 0.05)),
    () -> false,  () -> false));

    //Move robot to pick up cone
    new JoystickButton(driverController, Buttons.Y).whileTrue(new SwerveControllerCmd(robotDrive, () -> (0.15), () -> (0.0), 
      () -> (-MathMethods.speedMax2(0.045*limelightSubsystem.getTargetOffsetX(), 0.2, 0.05)),
      () -> false,  () -> false));

    //Align robot to AprilTag
    new JoystickButton(driverController, Buttons.Maria).whileTrue(new SwerveControllerCmd(robotDrive, () -> (MathMethods.speedMax2(0.05*limelightSubsystem.getTargetOffsetYLow(), 0.3, 0.01)), 
    () -> (-MathMethods.speedMax2(0.05*limelightSubsystem.getTargetOffsetXLow(), 0.3, 0.02)),
    () -> (0.0), ()->(false),  () -> false));
    
    new JoystickButton(driverController, Buttons.R3).onTrue(new InstantCommand(() -> robotDrive.resetEncoders()));

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return the autonomous command given by the drop-down selector in ShuffleBoard
    return autoChooser.getSelected();

  }

}
