// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCmd extends Command {

  private ShooterSubsystem SHOOTER_SUBSYSTEM; 

  private double topVelocity; 
  private double bottomVelocity; 

  

  /** Creates a new shooterVelocityCommand. */
  public ShooterCmd(ShooterSubsystem shooter, double topMotorVelocity, double bottomMotorVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topVelocity = topMotorVelocity; 
    this.bottomVelocity = bottomMotorVelocity; 

    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SHOOTER_SUBSYSTEM.setShooterVelocityMode();
    SHOOTER_SUBSYSTEM.setShooterRampRate(ShooterConstants.kShooterRampRate);
    SHOOTER_SUBSYSTEM.setTopPIDF(ShooterConstants.kTopShooterKp , ShooterConstants.kTopShooterKi, ShooterConstants.kTopShooterKd, ShooterConstants.kTopShooterKFf);
    SHOOTER_SUBSYSTEM.setBottomPIDF(ShooterConstants.kBottomShooterKp, ShooterConstants.kTopShooterKi, ShooterConstants.kBottomShooterKd, ShooterConstants.kBottomShooterKFf);

    SHOOTER_SUBSYSTEM.setTopEncoderOutputConstraints(ShooterConstants.kTopMax, ShooterConstants.kTopMin);
    SHOOTER_SUBSYSTEM.setBottomEncoderOutputConstraints(ShooterConstants.kTopMin, ShooterConstants.kTopMin);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    System.out.println("SHOOTER COMMAND!!"); 


    SHOOTER_SUBSYSTEM.setTopShooterVelocity((topVelocity));
    SHOOTER_SUBSYSTEM.setBottomShooterVelocity((bottomVelocity));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.setCoastMode();
    SHOOTER_SUBSYSTEM.setShooterVelocityMode();
    SHOOTER_SUBSYSTEM.setShooter(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
