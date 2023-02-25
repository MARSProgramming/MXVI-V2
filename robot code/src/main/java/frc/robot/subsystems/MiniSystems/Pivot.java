// This subsystem is to pivot the arm, and has constraints for how far the arm can turn. 

package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase{
    
    private final double kGearRatio = 81;
    private TalonFX pivot = new TalonFX(Constants.Pivot.motorID);
    private final double kRadianstoNativeUnits = 2048 / Math.PI / 2 * kGearRatio;
    private final DutyCycleEncoder mEncoder = new DutyCycleEncoder(0);
    private final ProfiledPIDController mController = new ProfiledPIDController(Constants.Pivot.kP, Constants.Pivot.kI, Constants.Pivot.kD, new TrapezoidProfile.Constraints(1, 1));
    
    public Pivot() {
        pivot.configFactoryDefault();
        pivot.setNeutralMode(NeutralMode.Brake);
        pivot.setInverted(true);
    
        mEncoder.setDistancePerRotation(Math.PI * 2);
        mEncoder.reset();
    }
    public void Run(double voltage) {
        if(mEncoder.getDistance() > Constants.Pivot.forwardLimit || mEncoder.getDistance() < Constants.Pivot.reverseLimit){
            pivot.set(ControlMode.PercentOutput, 0);
        }
        else{
            pivot.set(ControlMode.PercentOutput, voltage);
        }
    } 

    public void setpos(double angle) {
        Run(mController.calculate(mEncoder.getDistance(), angle * kRadianstoNativeUnits));
    }

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
            () -> {
                pivot.set(ControlMode.PercentOutput, d.getAsDouble());
              }, 
            () -> {
                pivot.set(ControlMode.PercentOutput, 0.0);
              }
            ).withName("Test Pivot");
    }

    public double distanceToSetpoint(double setpoint){
        return pivot.getSelectedSensorPosition() / kRadianstoNativeUnits - setpoint;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Position", mEncoder.getDistance());
    }
}


