package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Subsystems.DriveSubsystem;

public class turnToAngleCmd extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final PIDController turnController;
    private final double targetAngle;

    private static final double kAngleTolerance = 2.0;

    public turnToAngleCmd(DriveSubsystem driveSubsystem, double targetAngle) {

        this.driveSubsystem = driveSubsystem;
        this.targetAngle = targetAngle;

        turnController = new PIDController(1.15, 0, 0.1);
        turnController.setTolerance(kAngleTolerance);

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        double currentAngle = driveSubsystem.getHeading();
        double rotationSpeed = turnController.calculate(currentAngle);
        driveSubsystem.drive(0, 0, rotationSpeed, true, false);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, false);
    }

}