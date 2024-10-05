// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

import javax.naming.spi.ResolveResult;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.SwerveJoystickCmd;
import frc.robot.Commands.intakeCmd;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import java.util.List;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RunCommand;





public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  //private final Trigger driverLeftTrigger = new Trigger(() -> driverJoystick.getRawAxis(2) > OIConstants.kTriggerThreshold);//
  private final CommandGenericHID controllerPrimary = new CommandGenericHID(OIConstants.kDriverControllerPort) ;


  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
    ));
    
    
    configureBindings();
  }

  private void configureBindings() {
    controllerPrimary.axisGreaterThan(2, 0.1).toggleOnFalse(new intakeCmd(intakeSubsystem, IntakeConstants.intakeVelocity));
    controllerPrimary.axisGreaterThan(2, 0.1).toggleOnTrue(new intakeCmd(intakeSubsystem, IntakeConstants.intakeVelocity));


    // driverLeftTrigger.whileTrue(
    //   new intakeCmd(intakeSubsystem, IntakeConstants.intakeVelocity)
    // );

    //  driverLeftTrigger.whileFalse(
    //   new intakeCmd(intakeSubsystem, 0)
    // );

  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
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
        List.of(new Translation2d(0.71, 0)),//, new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.42, 0, new Rotation2d(0)),    //should be going in meters
        forwardConfig);

    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(1.42, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.71, 0)),//, new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),    //should be going in meters
        reverseConfig);

     Trajectory thirdTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.69, -1.5)),//, new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.37, -1.5, new Rotation2d(0)),    //should be going in meters
        reverseConfig);

    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand firstCommand = new SwerveControllerCommand(
        firstTrajectory,
        swerveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
    
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    SwerveControllerCommand secondCommand = new SwerveControllerCommand(
        secondTrajectory,
        swerveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
    
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    SwerveControllerCommand thirdCommand = new SwerveControllerCommand(
        thirdTrajectory,
        swerveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
    
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    
    // Reset odometry to the starting pose of the trajectory.
    swerveSubsystem.resetOdometry(firstTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    return firstCommand.andThen(() -> swerveSubsystem.stopModules()).andThen(Commands.waitSeconds(0.25)).andThen(secondCommand).andThen(() -> swerveSubsystem.stopModules()).andThen(Commands.waitSeconds(0.25)).andThen(thirdCommand).andThen(() -> swerveSubsystem.stopModules());

    //return null;
  }
}



