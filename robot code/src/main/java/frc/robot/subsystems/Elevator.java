package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * Double stage slanted elevator moving the manipulator up and down
 * 
 * 2 Motors - IDs x, y
 * Gear Ratio - 49:1(tentative)
 * Constraints - Cannot move too far up or down
 * 
 * Controls:
 * position control through moveToPosition() method
 * manual movement through movePercentOutput() method
 */

public class Elevator extends SubsystemBase{

    private TalonFX master;
    private TalonFX follower;
    private final double kGearRatio = 49;
    private final double inchesPerRotation = 1;
    private final double inchesToNativeUnits = 2048 * kGearRatio * inchesPerRotation;
    public Elevator(){
        master = new TalonFX(Constants.Elevator.followerMotorID);
        follower = new TalonFX(Constants.Elevator.masterMotorID);

        master.configFactoryDefault();
        follower.configFactoryDefault();

        master.configForwardSoftLimitThreshold(Constants.Elevator.forwardSoftLimitInches * inchesToNativeUnits);
        master.configForwardSoftLimitEnable(true);
        
        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
        
        master.config_kP(0, 0.0);
        master.config_kI(0, 0.0);
        master.config_kD(0, 0.0);

        follower.follow(master);
        
    }

    public void setPosition(double inches){
        master.set(ControlMode.Position, inches * inchesToNativeUnits);
    }

    public void setPercentOutput(double v){
        master.set(ControlMode.PercentOutput, v);
    }

}
