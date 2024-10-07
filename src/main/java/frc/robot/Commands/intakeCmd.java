// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.IntakeSubsystem;

public class intakeCmd extends Command {

  private IntakeSubsystem INTAKE_SUBSYSTEM; 

  private double intakeVelocity; 

  //private SlewRateLimiter intakeLimiter = new SlewRateLimiter(IntakeConstants.intakeSlewLimit); 

  /** Creates a new intakeVelocityCommand. */
  public intakeCmd(IntakeSubsystem intake, double velocity) {
    this.INTAKE_SUBSYSTEM = intake; 
    this.intakeVelocity = velocity;
    addRequirements(INTAKE_SUBSYSTEM); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    INTAKE_SUBSYSTEM.setIntakeVelocityMode();
    INTAKE_SUBSYSTEM.setRampRate(IntakeConstants.intakeRampRate);
    INTAKE_SUBSYSTEM.setIntakePIDF(IntakeConstants.intakeKp, IntakeConstants.intakeKi, IntakeConstants.intakeKd, IntakeConstants.intakeKFf);
    INTAKE_SUBSYSTEM.setIntakeEncoderOutputConstraints(IntakeConstants.intakeMin, IntakeConstants.intakeMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    INTAKE_SUBSYSTEM.setVelocityIntake((intakeVelocity)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.setCoastMode();
    INTAKE_SUBSYSTEM.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}