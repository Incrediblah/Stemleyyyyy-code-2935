// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoNoteDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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


public class secondCmd extends Command {
  /** Creates a new secondCmd. */
  private final DriveSubsystem m_robotDrive;
  public secondCmd(DriveSubsystem driveSubsystem) {
    m_robotDrive = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public Command secondCommand (){

    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);

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
    var thetaController = new ProfiledPIDController( AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand secondCommand = new SwerveControllerCommand(
      secondTrajectory,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);


      return(
        secondCommand) ;




  }

  // Called when the command is initially scheduled.
  
}
