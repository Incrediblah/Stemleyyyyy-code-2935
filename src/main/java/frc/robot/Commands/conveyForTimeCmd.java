
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Subsystems.ConveyorSubsystem;



public class conveyForTimeCmd extends Command {

  private ConveyorSubsystem CONVEYOR_SUBSYSTEM; 

  private double conveyorVelocity; 
  private int counter=0;
  private int target=0;

  
  /** Creates a new conveyerVelocityCommand. */
  public conveyForTimeCmd(ConveyorSubsystem conveyor, double velocity, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYOR_SUBSYSTEM = conveyor; 
    this.conveyorVelocity = velocity;
    target=(int)(seconds*50);
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
    if(counter< target)
    {counter++;
    } 
    CONVEYOR_SUBSYSTEM.setConveyorVelocity(conveyorVelocity*.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    CONVEYOR_SUBSYSTEM.setConveyorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter>=target ;
  }
}