package frc.robot.subsystems.MiniSystems;

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
import frc.robot.Constants;

public class Wrist extends SubsystemBase{

    private TalonFX mWristMotor;  

    private final double kRadiansToNativeUnits = 0;
    public Wrist(){
        mWristMotor = new TalonFX(Constants.Wrist.motorID);
        mWristMotor.configFactoryDefault();

        mWristMotor.setNeutralMode(NeutralMode.Brake);
        mWristMotor.config_kP(0, Constants.Wrist.kP);
        mWristMotor.config_kI(0, Constants.Wrist.kI);
        mWristMotor.config_kD(0, Constants.Wrist.kD);
    }
    
    public void setPosition(double pos){
        mWristMotor.set(ControlMode.Position, pos * kRadiansToNativeUnits);
    }

     public void setPercentOutput(double v){
        mWristMotor.set(ControlMode.PercentOutput, v);
     }
}
