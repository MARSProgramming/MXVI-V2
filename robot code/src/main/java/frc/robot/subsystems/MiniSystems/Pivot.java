// This subsystem is to pivot the arm, and has constraints for how far the arm can turn. 

package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase{
    
    private final double kGearRatio = 81;
    private TalonFX pivot = new TalonFX(Constants.Pivot.motorID);
    private final double kRadianstoNativeUnits = 2048 / Math.PI / 2 * kGearRatio;

    public Pivot() {
        pivot.configFactoryDefault();
        pivot.setNeutralMode(NeutralMode.Brake);
        pivot.config_kP(0, Constants.Pivot.kP);
        pivot.config_kI(0, Constants.Pivot.kI);
        pivot.config_kD(0, Constants.Pivot.kD);

        // Set constraints for how far forward/reverse the TalonFX can go
        pivot.configForwardSoftLimitThreshold(Constants.Pivot.forwardLimit, 0);
        pivot.configReverseSoftLimitThreshold(Constants.Pivot.reverseLimit, 0);
        pivot.configForwardSoftLimitEnable(true, 0);
        pivot.configReverseSoftLimitEnable(true, 0);
    }
    public void Run(double voltage) {
        pivot.set(ControlMode.PercentOutput, voltage);
    } 

    public void setpos(double angle) {
        pivot.set(ControlMode.Position, angle * kRadianstoNativeUnits);
    }
 
    public CommandBase runTestMode(DoubleSupplier d) {
        return run(
          () -> {
            pivot.set(ControlMode.PercentOutput, d.getAsDouble());
          }).withName("Test Pivot");
    }

    public double distanceToSetpoint(double setpoint){
        return pivot.getSelectedSensorPosition() / kRadianstoNativeUnits - setpoint;
    }
}


