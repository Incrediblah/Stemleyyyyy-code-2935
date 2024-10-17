// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class autoIntakeCmd extends SequentialCommandGroup{
  /** Creates a new autoIntakeCmd. */
  public autoIntakeCmd(ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelDeadlineGroup(
        new intakeSensorCmd( intakeSubsystem, IntakeConstants.intakeVelocity),
        new conveyorSensorCmd(conveyorSubsystem, ConveyorConstants.kConveyorVelocity)
        )
    );
  }

  // Called when the command is initially scheduled.
}