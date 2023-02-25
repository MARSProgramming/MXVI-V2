// This subsystem is the part of the robot that actually picks up cones and squares, with no sensor it will have to read va;ues to determine whether cargo has been picked up


package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.UtilityFunctions;



public class Grasper extends SubsystemBase{

    private TalonFX BeltController = new TalonFX(Constants.Grasper.motorID);
    public Grasper() {
        BeltController.configFactoryDefault();
        BeltController.setNeutralMode(NeutralMode.Coast);
        BeltController.setInverted(true);
    }

    public void RunGrasperEject(){
        BeltController.set(TalonFXControlMode.PercentOutput, -0.3);
    }

    public void RunGrasperStallcheck() {
        if (UtilityFunctions.isStalling(BeltController.getSelectedSensorPosition(), 500) && BeltController.getMotorOutputPercent() != 0) {
            BeltController.set(TalonFXControlMode.PercentOutput, 0);
        } else {
            BeltController.set(TalonFXControlMode.PercentOutput, 0.5);
        }
    }

    public void setPercentOutput(double v){
        BeltController.set(ControlMode.PercentOutput, v);
    }

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
          () -> {
            RunGrasperStallcheck();
          },
          () -> {
            BeltController.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Grasper");
    }
    
}