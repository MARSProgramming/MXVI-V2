package frc.robot.subsystems;

//WristSubsystem: 
//Position Control via REV Encoder (limits?)
//Controls Y axis of end affecter (up/down)
//Gear ratio:
//ID:
//
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristMotor extends SubsystemBase{

    private TalonFX mWristMotor;  
    private final double kRadianstoNativeUnits = 0;
    public WristMotor(){

        mWristMotor = new TalonFX(13);
        mWristMotor.configFactoryDefault();
        mWristMotor.setNeutralMode(NeutralMode.Brake);
        mWristMotor.config_kP(0, 0.0);
        mWristMotor.config_kI(0, 0.0);
        mWristMotor.config_kD(0, 0.0);
        mWristMotor.set(ControlMode.Position, 0);
       
    }
    
    // position control
    
    public void setWrsitPosition(double v){
        mWristMotor.set(ControlMode.Position, v * kRadianstoNativeUnits);
    }

     public void runWristPOutput(double v){
        mWristMotor.set(ControlMode.PercentOutput, v);
     }
}
