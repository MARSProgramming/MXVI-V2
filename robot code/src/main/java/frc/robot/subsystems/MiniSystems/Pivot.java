// This subsystem is to pivot the arm, and has constraints for how far the arm can turn. 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Pivot {
    private TalonFX pivot = new TalonFX(0);
    private final double kRadianstoNativeUnits = 0;

    public Pivot() {
        pivot.configFactoryDefault();
        pivot.setNeutralMode(NeutralMode.Brake);
        pivot.config_kP(0, 0.0);
        pivot.config_kI(0, 0.0);
        pivot.config_kD(0, 0.0);

        // Set constraints for how far forward/reverse the TalonFX can go
        pivot.configForwardSoftLimitThreshold(1024, 0);
        pivot.configReverseSoftLimitThreshold(-1024, 0);
        pivot.configForwardSoftLimitEnable(true, 0);
        pivot.configReverseSoftLimitEnable(true, 0);
    }
    public void Run(double voltage) {
        pivot.set(ControlMode.PercentOutput, voltage);
    } 

    public void setpos(double angle) {
        pivot.set(ControlMode.Position, angle * kRadianstoNativeUnits);
    }
 
}


