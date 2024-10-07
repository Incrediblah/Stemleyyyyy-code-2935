// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands.ConveyorCommands;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Subsystems.ConveyorSubsystem;

/** Add your docs here. */
public class conveyorSensorCmd extends Command{

    private ConveyorSubsystem CONVEYOR_SUBSYSTEM; 
    private double initTime; 
    private double kConveyorVelocity; 


    public conveyorSensorCmd(ConveyorSubsystem conveyer, double velocity) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.CONVEYOR_SUBSYSTEM = conveyer; 
        this.kConveyorVelocity = velocity; 
        addRequirements(CONVEYOR_SUBSYSTEM);
      }

       // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.currentTimeMillis(); 
    CONVEYOR_SUBSYSTEM.setConveyorVelocityMode();
    System.out.println("SENSOR COMMAND RUNNING!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYOR_SUBSYSTEM.setConveyorVelocity(kConveyorVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SENSOR COMMAND ENDED");
    CONVEYOR_SUBSYSTEM.stop();
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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

