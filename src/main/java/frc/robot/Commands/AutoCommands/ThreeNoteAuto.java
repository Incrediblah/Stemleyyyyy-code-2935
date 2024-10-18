package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Subsystems.*;

import java.util.List;

public class ThreeNoteAuto extends SequentialCommandGroup {
  private DriveSubsystem m_robotDrive;
   
    public ThreeNoteAuto(
     DriveSubsystem m_robotdrive,
     ShooterSubsystem shooterSubsystem,
     IntakeSubsystem intakeSubsystem,
     ConveyorSubsystem conveyorSubsystem){
        
        this.m_robotDrive = m_robotdrive;
        addRequirements(m_robotdrive);

       

           
        
       
        // Create trajectory configurations
        TrajectoryConfig forwardConfig = createForwardConfig();
        TrajectoryConfig reverseConfig = createReverseConfig();

        // Generate all trajectories
        Trajectory firstTrajectory = generateFirstTrajectory(forwardConfig);
        Trajectory secondTrajectory = generateSecondTrajectory(reverseConfig);
        Trajectory thirdTrajectory = generateThirdTrajectory(forwardConfig);
        
        // Create PID controllers
        var xController = new PIDController(AutoConstants.kPXController, 0, 0.5);
        var yController = new PIDController(AutoConstants.kPYController, 0, 0.5);
        var thetaController = createThetaController();

        // Create swerve controller commands
        SwerveControllerCommand firstCommand = createSwerveCommand(firstTrajectory, xController, yController, thetaController);
        SwerveControllerCommand secondCommand = createSwerveCommand(secondTrajectory, xController, yController, thetaController);
        SwerveControllerCommand thirdCommand = createSwerveCommand(thirdTrajectory, xController, yController, thetaController);
        
        // Add commands to the sequential command group
        addCommands(
             new SequentialCommandGroup(
        new InstantCommand(()-> m_robotDrive.resetOdometry(firstTrajectory.getInitialPose())),
    
            //new ShootForTimeCmd(ShooterSubsystem, ShooterConstants.subwooferTopVelocity,ShooterConstants.subwooferBottomVelocity,25),
        
                new ParallelCommandGroup(
            new SequentialCommandGroup(
             new InstantCommand(() -> m_robotDrive.stopModules()),
            new ShootForTimeCmd(shooterSubsystem, ShooterConstants.subwooferTopVelocity,ShooterConstants.subwooferBottomVelocity,2)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1.35),
                new  conveyForTimeCmd(conveyorSubsystem, ConveyorConstants.kConveyorVelocity,1)
            )),

        
       
        new ParallelCommandGroup(
        firstCommand,
        new conveyorSensorCmd(conveyorSubsystem,ConveyorConstants.kConveyorVelocity),
        new intakeForTimeCmd(intakeSubsystem,IntakeConstants.intakeVelocity,3)
        ),
        new InstantCommand(() -> m_robotDrive.stopModules()),
        secondCommand,
        new ParallelCommandGroup(
            new SequentialCommandGroup(
             new InstantCommand(() -> m_robotDrive.stopModules()),
            new ShootForTimeCmd(shooterSubsystem, ShooterConstants.subwooferTopVelocity,ShooterConstants.subwooferBottomVelocity,2)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1.35),
                new  conveyForTimeCmd(conveyorSubsystem, ConveyorConstants.kConveyorVelocity,1)
            )
        ),
        new InstantCommand(() -> m_robotDrive.stopModules()),
   

        new ParallelCommandGroup(
        thirdCommand,
        new conveyorSensorCmd(conveyorSubsystem,ConveyorConstants.kConveyorVelocity),
        new intakeForTimeCmd(intakeSubsystem,IntakeConstants.intakeVelocity, 4)
        ),
        
        new turnToAngleCmd(m_robotDrive, -15),
        
         
        new ParallelCommandGroup(
            new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.stopModules()),
            new ShootForTimeCmd(shooterSubsystem, ShooterConstants.autolineTopVelocity,ShooterConstants.autolineBottomVelocity,2)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                new  conveyForTimeCmd(conveyorSubsystem, ConveyorConstants.kConveyorVelocity,1)
            )
        )));
    }

    private TrajectoryConfig createForwardConfig() {
        return new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    }

    private TrajectoryConfig createReverseConfig() {
        return new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);
    }

    private ProfiledPIDController createThetaController() {
        var controller = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }

    

    private Trajectory generateFirstTrajectory(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),
            new Pose2d(1.5, 0, new Rotation2d(0)),
            config
        );
    }

    private Trajectory generateSecondTrajectory(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.5, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0), new Translation2d(0.5, 0)),
            new Pose2d(0, 0, new Rotation2d(0)),
            config
        );
    }

    private Trajectory generateThirdTrajectory(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0.5, -1.38)),
            new Pose2d(1.65, -1.38, new Rotation2d(0)),
            config
        );
    }

    

    private SwerveControllerCommand createSwerveCommand(
            Trajectory trajectory,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController) {
        return new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive
        );
    }
}