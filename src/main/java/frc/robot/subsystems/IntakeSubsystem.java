// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.CANSparkFlex;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSubsystem extends SubsystemBase {

  private final String INTAKE_PREFIX = "SmartDashboard/Intake"; 

  private CANSparkFlex intake_motor = new CANSparkFlex(IntakeConstants.intakeMotorCANId, MotorType.kBrushless); 
  private RelativeEncoder intake_encoder = intake_motor.getEncoder();

  /** Creates a new INTAKE_SUBSYSTEM. */
  public IntakeSubsystem() { 
    intake_motor.restoreFactoryDefaults();    // restore motor settings to factory defult
    intake_motor.setSmartCurrentLimit(50);    // Set the current limit of the motor
    intake_motor.setInverted(false);    // Set motor to not inverted
    intake_encoder.setPosition(0);    // Set encoder position to zero
    intake_motor.burnFlash();   // Burn and flash the new settings to the motor controller

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(INTAKE_PREFIX + "has intaked", has_intaked()); 
  }

  // SET TO BRAKE MODE 
  public void setBrakeMode(){
    intake_motor.setIdleMode(IdleMode.kBrake); 
  }

  // SET TO COAST MODE 
  public void setCoastMode(){
    intake_motor.setIdleMode(IdleMode.kCoast); 
  }

  // RESET ENCODER
  public void resetEncoders(){
    intake_encoder.setPosition(0);  
  }

  public double getintake_encoderPosition(){
    return intake_encoder.getPosition(); 
  }

  public double getintake_encoderVelocity(){
    return intake_encoder.getVelocity();     
  }


  public double getIntake_EncoderCurrent(){
    return intake_motor.getOutputCurrent(); 
  }

  public boolean has_intaked(){
    if(getIntake_EncoderCurrent() > 15){ 
      return true; 
    }
    else{
      return false; 
    } 
  }

  public void setIntakePIDF(double p, double i, double d, double f){
    intake_motor.getPIDController().setP(p); 
    intake_motor.getPIDController().setI(i); 
    intake_motor.getPIDController().setD(d); 
    intake_motor.getPIDController().setFF(f); 
  }

  public void setIntakeEncoderOutputConstraints(double min, double max){
    intake_motor.getPIDController().setOutputRange(min, max); 
  }

  public void setIntakeVelocityMode(){
    intake_motor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setShooterPowerMode(){
    intake_motor.getPIDController().setReference(0, ControlType.kCurrent); 
  }

  public void setVelocityIntake(double topVelocity){
    intake_motor.getPIDController().setReference(topVelocity, ControlType.kVelocity); 
  }

  public void setRampRate(double ramp){
    intake_motor.setClosedLoopRampRate(ramp);
  }

  public void setIntake(double intakeSpeed){
    intake_motor.set(intakeSpeed);
  }

  public void stop(){
    intake_motor.stopMotor();
  }

}