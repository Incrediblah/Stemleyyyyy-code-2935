package frc.robot.Subsystems;
import frc.robot.Constants.ConveyorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StatusVariables;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConveyorSubsystem extends SubsystemBase {
    
    private CANSparkMax m_Conveyor = new CANSparkMax(ConveyorConstants.kConveyorCANId, MotorType.kBrushless);
    private RelativeEncoder m_ConveyorEncoder = m_Conveyor.getEncoder();

    //more conveyer sensor stuff
    private DigitalInput conveyorSwitch1 = new DigitalInput(ConveyorConstants.switchOnePort); 

    public ConveyorSubsystem() {

        m_Conveyor.restoreFactoryDefaults();
        m_Conveyor.setSmartCurrentLimit(50);
        m_Conveyor.setInverted(false);
        m_ConveyorEncoder.setPosition(0);
        m_Conveyor.burnFlash();

    }

    public void periodic() {

    }

    public void setConveyorPIDF(double p, double i, double d, double f){
        m_Conveyor.getPIDController().setP(p); 
        m_Conveyor.getPIDController().setI(i); 
        m_Conveyor.getPIDController().setD(d); 
        m_Conveyor.getPIDController().setFF(f); 
    } 
    
    public void setConveyorEncoderOutputConstraints(double min, double max){
        m_Conveyor.getPIDController().setOutputRange(min, max); 
    }

    public void setRampRate(double ramp){
        m_Conveyor.setClosedLoopRampRate(ramp);
    }

    public void setConveyorVelocityMode() {
        m_Conveyor.getPIDController().setReference(0, ControlType.kVelocity);
    }

    public void setConveyorBrakeMode() {
        m_Conveyor.setIdleMode(IdleMode.kBrake);
    }

    public void setConveyorCoastMode() {
        m_Conveyor.setIdleMode(IdleMode.kCoast);
    }

    public void resetConveyorEncoders() {
        m_ConveyorEncoder.setPosition(0);
    }

      public void setConveyorVelocity(double conveyorVelocity) {
        m_Conveyor.getPIDController().setReference(conveyorVelocity, ControlType.kVelocity);
    }

    public double getConveyorEncoderPosition() {
        return m_ConveyorEncoder.getPosition();
    }

    public double getConveyorEncoderVelocity() {
        return m_ConveyorEncoder.getVelocity();
    }

    public void setConveyorCoastMode(double conveyorVelocity) {
        m_Conveyor.getPIDController().setReference(conveyorVelocity, ControlType.kVelocity);
    }

    //conveyer sensor if active or not
    public boolean getConveyorSwitchOneValue(){
        StatusVariables.conveyerSwitchOneStatus = conveyorSwitch1.get();
        return conveyorSwitch1.get(); 
      }

    public void stop() {
        m_Conveyor.stopMotor();
    }

}
