package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 18; // 15 feels nice // was 4.8
        public static final double kMaxAngularSpeed = 15 * Math.PI;// 8 feels nice // was 2 // radians per second

        public static final double kDirectionSlewRate = 7; // radians per second
        public static final double kMagnitudeSlewRate = 3.5; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

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


        public static final boolean kGyroReversed = true;
  }

    public static final class ModuleConstants {

    // Driving Gear for configuring the maxswerve
    // 14T high speed, 13T medium speed, 12T low speed
        public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

    // Calculations for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRPM = 6784;
        public static final double kDrivingMotorFreeSpeedRps = kDrivingMotorFreeSpeedRPM / 60;
        public static final double kWheelDiameterInches = 3;
        public static final double kWheelDiameterMeters = kWheelDiameterInches * 0.0254;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 Teeth on the bevel gear, 22 teeth on first stage spur, 15T on bevel pinon
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    // Encoder Position Factor
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;   // meters
        
    // Encoder Velocity Factor
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60;    // meters per second

    // Turning Encoder Position Factor
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI);   // radians

    // Turning Encoder Velocity Factor
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60;  // radians per second

    // Turning Encoder Position PID
        public static final double kTurningEncoderPositionPIDMinInput = 0;  // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;  // radians

        public static final double kDrivingP = 0.01;    // was 0.04
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0.01;   // was (1 / kDriveWheelFreeSpeedRps)
        public static final double kDrivingMinOutput = -0.4;
        public static final double kDrivingMaxOutput = 0.4;

        public static final double kTurningP = 1.1;     // was 1.1
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;      // DO NOT INCREASE
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    // Setting Current Limits
        public static final int kDrivingMotorCurrentLimit = 60;     // amps
        public static final int kTurningMotorCurrentLimit = 20;     // amps

   }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final double kTriggerThreshold = 0.5;

        public static final double kDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        

        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; //DriveConstants.kMaxAngularSpeed / 10;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //Math.PI / 4;

        public static final double kPXController = 100;
        public static final double kPYController = 100;
        public static final double kPThetaController = 100;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IntakeConstants {
        public static final int intakeMotorCANId = 13; 
        public static final double intakeSpeed = 0.35; // amps
    
        public static final int neoVortexRPM = 6784; 
    
    
        
        
    
         // INTAKE
         public static double intakeKp = 6e-5; 
         public static double intakeKi = 0.0000; 
         public static double intakeKd = 0; 
         public static double intakeKIz = 6e-5; 
         public static double intakeKFf = 0.000015;
     
         // SHOOTER MOTOR OUTPUT CONSTRAINTS
         public static double intakeMax = 1; 
         public static double intakeMin = -1; 
     
         // SHOOTER MOTOR SLEW RATE LIMITERS 
         public static double intakeSlewLimit = 2; 
    
         // SHOOTER MOTOR RAMP RATE 
         public static double intakeRampRate = 0.25;

    
        public static final double intakeVelocity = intakeSpeed * neoVortexRPM;
    }

}
