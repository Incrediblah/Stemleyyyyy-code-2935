

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Subsystems.ConveyorSubsystem;



public class conveyorSensorCmd extends Command {

  private ConveyorSubsystem CONVEYOR_SUBSYSTEM; 

  private double conveyorVelocity; 

  
  /** Creates a new conveyerVelocityCommand. */
  public conveyorSensorCmd(ConveyorSubsystem conveyor, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYOR_SUBSYSTEM = conveyor; 
    this.conveyorVelocity = velocity;
    addRequirements(CONVEYOR_SUBSYSTEM); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CONVEYOR_SUBSYSTEM.setConveyorVelocityMode();
    CONVEYOR_SUBSYSTEM.setRampRate(ConveyorConstants.conveyerRampRate);
    CONVEYOR_SUBSYSTEM.setConveyorPIDF(ConveyorConstants.kConveyerKp, ConveyorConstants.kConveyerKi, ConveyorConstants.kConveyerKd, ConveyorConstants.kConveyerKFf);

    CONVEYOR_SUBSYSTEM.setConveyorEncoderOutputConstraints(ConveyorConstants.kConveyerMin, ConveyorConstants.kConveyerMax);
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYOR_SUBSYSTEM.setConveyorVelocity((conveyorVelocity*1.5)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYOR_SUBSYSTEM.setConveyorCoastMode();
   // CONVEYOR_SUBSYSTEM.setConveyerPowerMode();
    CONVEYOR_SUBSYSTEM.setConveyorVelocity(0);
  }

  
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     if(CONVEYOR_SUBSYSTEM.getConveyorSwitchOneValue() == false){
       return true; 
     }
     else{
       return false; 
     }
   }
}