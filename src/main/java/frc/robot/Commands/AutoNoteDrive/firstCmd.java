// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoNoteDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Commands.conveyorSensorCmd;
import frc.robot.Commands.intakeCmd;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

import frc.robot.Subsystems.DriveSubsystem;



public class firstCmd extends Command {
  /** Creates a new autoDriveCmd. */
  private final DriveSubsystem m_robotDrive;
  //private final IntakeSubsystem intake;
   //private final ConveyorSubsystem conveyor;
  

  public firstCmd (DriveSubsystem driveSubsystem){
    m_robotDrive = driveSubsystem;
   // intake = intakeSubsystem;
    //conveyor = conveyorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    
  }
     public Command firstCommand (){
  TrajectoryConfig forwardConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

   


// An example trajectory to follow. All units in meters.
Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),//, new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(1.5, 0, new Rotation2d(0)),    //should be going in meters
    forwardConfig);

   




// Create PID Controllers
var xController = new PIDController(AutoConstants.kPXController, 0, 0.5);
var yController = new PIDController(AutoConstants.kPYController, 0, 0.5);
var thetaController = new ProfiledPIDController( AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
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

   

  

        return 
        
        (firstCommand);
        
        
 }}





  // Called when the command is initially scheduled.
  