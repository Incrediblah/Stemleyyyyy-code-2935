// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.StatusVariables;


public class RumbleCmd extends Command {
  private final XboxController m_controllerPrimary;


  /** Creates a new RumbleCommand. */
  public RumbleCmd(XboxController controllerPrimary) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controllerPrimary = controllerPrimary;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(StatusVariables.ConveyorSwitchTwoStatus==false)  {
      m_controllerPrimary.setRumble(RumbleType.kLeftRumble, 2.0);
      m_controllerPrimary.setRumble(RumbleType.kRightRumble, 2.0);
    } else {
      m_controllerPrimary.setRumble(RumbleType.kLeftRumble, 0.0);
      m_controllerPrimary.setRumble(RumbleType.kRightRumble, 0.0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controllerPrimary.setRumble(RumbleType.kLeftRumble, 0.0);
      m_controllerPrimary.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
