package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    
    //Setup Shooter Motors
    private CANSparkFlex m_topShooter = new CANSparkFlex(ShooterConstants.kTopShooterId, MotorType.kBrushless);
    private CANSparkFlex m_bottomShooter = new CANSparkFlex(ShooterConstants.kBottomShooterId, MotorType.kBrushless);

    // Shooter Encoder Setup
    private RelativeEncoder m_topShooterEncoder = m_topShooter.getEncoder();
    private RelativeEncoder m_bottomShooterEncoder = m_bottomShooter.getEncoder();

    // Initialize motors
    public ShooterSubsystem() {

        m_topShooter.restoreFactoryDefaults();
        m_bottomShooter.restoreFactoryDefaults();

        m_topShooter.setSmartCurrentLimit(50);
        m_bottomShooter.setSmartCurrentLimit(50);

        m_topShooter.setInverted(false);
        m_bottomShooter.setInverted(true);

        m_topShooterEncoder.setPosition(0);
        m_bottomShooterEncoder.setPosition(0);

        m_topShooter.burnFlash();
        m_bottomShooter.burnFlash();
        
    }

    public void periodic() {

        SmartDashboard.putNumber("Shooter Top Roller Speed", getTopShooterVelocity());
        SmartDashboard.putNumber("Shooter Bottom Roller Speed", getBottomShooterVelocity());

    }

    public void setBrakeMode() {
        m_topShooter.setIdleMode(IdleMode.kBrake);
        m_bottomShooter.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode() {
        m_topShooter.setIdleMode(IdleMode.kCoast);
        m_bottomShooter.setIdleMode(IdleMode.kCoast);
    }

    public void resetEncoders() {
        m_topShooterEncoder.setPosition(0);
        m_bottomShooterEncoder.setPosition(0);
    }

    public double getTopShooterPosition() {
        return m_topShooterEncoder.getPosition();
    }

    public double getBottomShooterPosition() {
        return m_bottomShooterEncoder.getPosition();
    }

    public double getTopShooterVelocity() {
        return m_topShooterEncoder.getVelocity();
    }

    public double getBottomShooterVelocity() {
        return m_bottomShooterEncoder.getVelocity();
    }

    public void setTopPIDF(double p, double i, double d, double f){
        m_topShooter.getPIDController().setP(p); 
        m_topShooter.getPIDController().setI(i); 
        m_topShooter.getPIDController().setD(d); 
        m_topShooter.getPIDController().setFF(f); 
    }

    public void setBottomPIDF(double p, double i, double d, double f){
        m_bottomShooter.getPIDController().setP(p);
        m_bottomShooter.getPIDController().setI(i);
        m_bottomShooter.getPIDController().setD(d);
        m_bottomShooter.getPIDController().setFF(f);
    }

    public void setShooterRampRate(double ramp) {
        m_topShooter.setClosedLoopRampRate(ramp);
        m_bottomShooter.setClosedLoopRampRate(ramp);
    }

    public void setTopEncoderOutputConstraints(double min, double max){
        m_topShooter.getPIDController().setOutputRange(min, max); 
    }
    
    public void setBottomEncoderOutputConstraints(double min, double max){
        m_bottomShooter.getPIDController().setOutputRange(min, max); 
    } 
    
    public void setShooterVelocityMode(){
        m_topShooter.getPIDController().setReference(0, ControlType.kVelocity); 
        m_bottomShooter.getPIDController().setReference(0, ControlType.kVelocity); 
    }

    public void setTopShooterVelocity(double topShooterVelocity) {
        m_topShooter.getPIDController().setReference(topShooterVelocity, ControlType.kVelocity);
    }

    public void setBottomShooterVelocity(double bottomShooterVelocity) {
        m_bottomShooter.getPIDController().setReference(bottomShooterVelocity, ControlType.kVelocity);
    }

    public void stopShooter() {
        m_topShooter.stopMotor();
        m_bottomShooter.stopMotor();
    }

}
