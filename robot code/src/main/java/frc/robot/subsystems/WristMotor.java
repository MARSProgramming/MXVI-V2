package frc.robot.subsystems;

//WristSubsystem: 
//Position Control via REV Encoder (limits?)
//Controls Y axis of end affecter (up/down)
//Gear ratio:
//ID:
//
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristMotor extends SubsystemBase{

    private TalonFX mWristMotor;  

    public WristMotor(){

        mWristMotor = new TalonFX(13);
        mWristMotor.configFactoryDefault();
        mWristMotor.setNeutralMode(NeutralMode.Brake);
        mWristMotor.config_kP(0, kP);
        mWristMotor.config_kI(0, kI);
        mWristMotor.config_kD(0, kD);
        mWristMotor.set(ControlMode.Position, 0);
        int radians2NativeUnits;
        radians2NativeUnits = 0; 

    }
    
    // position control
    
    public void setWrsitPosition(double v){
        mWristMotor.set(ControlMode.Position, v * radians2NativeUnits);
    }

     public void runWristPOutput(double v){
        mWristMotor.set(ControlMode.PercentOutput, v);
     }
}
