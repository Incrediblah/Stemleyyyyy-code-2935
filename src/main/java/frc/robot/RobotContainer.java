// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Commands.LedCmd;
import frc.robot.Commands.ShooterCmd;
import frc.robot.Commands.intakeCmd;
import frc.robot.Commands.ConveyorCommands.conveyorSensorCmd;
import frc.robot.Commands.ConveyorCommands.conveyorCmd;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import java.util.List;
import edu.wpi.first.wpilibj2.command.RunCommand;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
   private final ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();
   private final ConveyorSubsystem ConveyorSubsystem = new ConveyorSubsystem();
   private final LedSubsystem LedSubsystem = new LedSubsystem();
   


  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final JoystickButton driverStartButton = new JoystickButton(m_driverController, 7);
  private final CommandGenericHID controllerPrimary = new CommandGenericHID(OIConstants.kDriverControllerPort);
  private final JoystickButton PRIMARY_BUTTON_A = new JoystickButton(m_driverController,1);   
  private final JoystickButton PRIMARY_BUTTON_X = new JoystickButton(m_driverController,OIConstants.BUTTON_X_PORT);
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    LedSubsystem.setDefaultCommand(new LedCmd (LedSubsystem));

    // Configure the button bindings
    configureBindings();
    

    // Configure default commands
    
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
  }


 //mapping commands to buttons below
  private void configureBindings() {
//intake and conveyor sensor control
    controllerPrimary.axisGreaterThan(2, 0.1).toggleOnFalse
    (new ParallelCommandGroup(
        new intakeCmd(IntakeSubsystem, 0),
        new conveyorSensorCmd(ConveyorSubsystem, 0)
         ));
    controllerPrimary.axisGreaterThan(2, 0.1).toggleOnTrue(
        new ParallelCommandGroup(
        new intakeCmd(IntakeSubsystem, IntakeConstants.intakeVelocity),
        new conveyorSensorCmd(ConveyorSubsystem, ConveyorConstants.kConveyorVelocity)
        ));
 //conveyer without sensor control
    controllerPrimary.axisGreaterThan(3,0.1).toggleOnTrue(
    new conveyorCmd(ConveyorSubsystem, ConveyorConstants.kConveyorVelocity)
    );  
    controllerPrimary.axisGreaterThan(3,0.1).toggleOnFalse(
        new conveyorCmd(ConveyorSubsystem,0)
    );   
 //shooter subwoofer control
    PRIMARY_BUTTON_A.onTrue(new ShooterCmd(ShooterSubsystem, ShooterConstants.subwooferTopVelocity, ShooterConstants.subwooferBottomVelocity));
    PRIMARY_BUTTON_A.onFalse(new ShooterCmd(ShooterSubsystem,0, 0 ));

   // PRIMARY_BUTTON_A.onTrue(new LedCmd(LedSubsystem));
    //PRIMARY_BUTTON_A.onFalse(Commands.none());

 //zero robot control
     driverStartButton.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Create config for trajectory
    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);

    // An example trajectory to follow. All units in meters.
    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),//, new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 0, new Rotation2d(0)),    //should be going in meters
        forwardConfig);

        Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(1.5, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(0.5, 0)),//, new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),    //should be going in meters
        reverseConfig);

    


    // Create PID Controllers
    var xController = new PIDController(AutoConstants.kPXController, 0, 0.5);
    var yController = new PIDController(AutoConstants.kPYController, 0, 0.5);
    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand firstCommand = new SwerveControllerCommand(
        firstTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        SwerveControllerCommand secondCommand = new SwerveControllerCommand(
            secondTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    
    // Run path following command, then stop at the end.
    return new InstantCommand(() -> m_robotDrive.resetOdometry(firstTrajectory.getInitialPose()))
    .andThen(new ParallelDeadlineGroup(firstCommand, new conveyorCmd(ConveyorSubsystem,ConveyorConstants.kConveyorVelocity)))
    .andThen(Commands.waitSeconds(0.15))
    .andThen(() -> m_robotDrive.stopModules())
    .andThen(new ParallelDeadlineGroup(secondCommand, new ShooterCmd(ShooterSubsystem,ShooterConstants.subwooferTopVelocity, ShooterConstants.subwooferBottomVelocity)))
    .andThen(Commands.waitSeconds(0.15))
    .andThen(() -> m_robotDrive.stopModules())
    .andThen(new ShooterCmd(ShooterSubsystem, 0, 0));
    
    
    
    
    

  }
    
     

}
