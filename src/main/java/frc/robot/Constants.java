// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.POVButton;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 5; // radians per second
    public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 1.4; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot  
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 6;


    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int BUTTON_A_PORT = 1;
    public static final int BUTTON_X_PORT = 3;
    public static final int BUTTON_B_PORT = 2;
    public static final int DpadUp = 0;
    public static final int DpadRight = 90;
    public static final int DpadDown = 180;
    public static final int DpadLeft = 270;
    public static final double TriggerThreshold = 0.5; 
    public static final int StartButton=7;
    public static final int BUTTON_LB_PORT = 5;

    

   
    
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2.75;
    public static final double kPYController = 2.75;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorCANId = 13; 
    //public static final double intakeSpeedPercent = 1; // amps
    public static final int neoRPM = 5676; 
    public static final double intakeVelocity = neoRPM;
    public static final double intakebackVelocity = -neoRPM;
    




   

    // INTAKE PID Controller
     public static double intakeKp = 6e-5; 
     public static double intakeKi = 0.0000; 
     public static double intakeKd = 0; 
     public static double intakeKIz = 6e-5; 
     public static double intakeKFf = 0.000015;

     public static double intakeMax = 1; 
     public static double intakeMin = -1; 
 
     // SHOOTER MOTOR SLEW RATE LIMITERS 
     public static double intakeSlewLimit = 2; 

     // SHOOTER MOTOR RAMP RATE 
     public static double intakeRampRate = 0.25;

  }

     

  public static final class ShooterConstants {

    public static final int kTopShooterId = 11;
    public static final int kBottomShooterId = 12; 
    public static final int neoVortexRPM = 6784;

  

    //______________________SHOOTER SPEEDS______________________//

    // Subwoofer Speed
    public static final double subwooferTopVelocity = neoVortexRPM * 0.23; //0.23
    public static final double subwooferBottomVelocity = neoVortexRPM;

    //AUTOLINE SHOT
    public static final double autolineTopVelocity = neoVortexRPM * 0.67 ; 
    public static final double autolineBottomVelocity = neoVortexRPM * 0.5; 

    //Pass speed
    public static final double passTopVelocity=neoVortexRPM*0.65;
    public static final double passBottomVelocity=neoVortexRPM*0.75;

   

    //______________________SHOOTER MOTOR SETUP______________________//

    // TOP SHOOTER MOTOR PID 
    public static double kTopShooterKp = 6e-5; 
    public static double kTopShooterKi = 0.0000; 
    public static double kTopShooterKd = 0; 
    public static double kTopShooterKIz = 6e-5; 
    public static double kTopShooterKFf = 0.000015;

    // BOTTOM SHOOTER MOTOR  PID
    public static double kBottomShooterKp = 6e-5; 
    public static double kBottomShooterKi = 0.0000; 
    public static double kBottomShooterKd = 0; 
    public static double kBottomShooterKIz = 6e-5; 
    public static double kBottomShooterKFf = 0.000015;

    // SHOOTER MOTOR OUTPUT CONSTRAINTS
    public static double kTopMax = 1; 
    public static double kTopMin = -1; 
     
    public static double kBottomMax = 1; 
    public static double kBottomMin = -1; 
     
    // SHOOTER MOTOR SLEW RATE LIMITERS 
    public static double kTopSlewLimit = 2; 
    public static double kBottomSlewLimit = 2; 
     
    // SHOOTER MOTOR RAMP RATE 
    public static double kShooterRampRate = 1;

  }

  public static final class ConveyorConstants {

    public static final int kConveyorCANId = 10;
    public static final int neoRPM = 5676;

    public static final double kConveyorVelocity = neoRPM;

    public static double kConveyerKp = 6e-5; 
    public static double kConveyerKi = 0.0000; 
    public static double kConveyerKd = 0; 
    public static double kConveyerKIz = 6e-5; 
    public static double kConveyerKFf = 0.000015;

    public static double kConveyerMax = 1; 
    public static double kConveyerMin = -1; 

    public static double conveyerSlewLimit = 2; 

    public static double conveyerRampRate = 0.25;

    //conveyer sensor stuff
    public static final int switchOnePort = 0; 
    public static final int switchTwoport = 1;

  }

  public static class LedConstants {

    public static final int ledPort = 0; 

  
public static final int ledLength = 60; 
  
// INDIVIDUAL COLOUR CODES 
public static final int[] greenColourCode = {0, 255, 0}; 
public static final int[] blueColourCode = {0, 0, 255}; 
public static final int[] redColourCode = {255, 0, 0}; 
public static final int[] orangeColourCode = {255, 25, 0}; 
public static final int[] whiteColourCode = {255, 125, 50}; 
public static final int[] vermillionColourCode = {255, 255, 255}; 
public static final int[] purpleColourCode = {200, 0, 200}; 
public static final int[] yellowColourCode = {200, 150, 0}; 
  }

  public static class StatusVariables{
    public static boolean ConveyorSwitchOneStatus;
    public static boolean ConveyorSwitchTwoStatus;
  

  }

  

}
