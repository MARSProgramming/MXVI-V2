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



public class Grasper extends SubsystemBase{

    private TalonFX BeltController = new TalonFX(Constants.Grasper.motorID);
    private double[] PositionTable = new double[]{5000,5000,5000,5000,5000,5000,5000,5000,5000,5000};
    private int count = 0;
    public Grasper() {
        BeltController.configFactoryDefault();
        BeltController.setNeutralMode(NeutralMode.Brake);
    }

    public void RunGrasperIntake(){
        double Average = 0;
        double Sdev = 0;
        for(int i = 0; i <= PositionTable.length; i++){
            Average += PositionTable[i];
        }
          Average = Average / PositionTable.length;

        for(int i = 0; i <= PositionTable.length; i++){
            Sdev += Math.pow(PositionTable[i] - Average, 2);
        }
        Sdev = Math.sqrt(Sdev);
      

        if(Math.abs(BeltController.getSelectedSensorPosition() - Average) < Sdev*2){
            return;
        }

        BeltController.set(TalonFXControlMode.PercentOutput, 0.3);

        PositionTable[count] = BeltController.getSelectedSensorPosition();

        if(count % PositionTable.length == 0 && !(count == 0)){
            count = 0;
        }
        else{
            count++;
        }
    }

    public void RunGrasperEject(){
        BeltController.set(TalonFXControlMode.PercentOutput, -0.3);
    }

    public void setPercentOutput(double v){
        BeltController.set(ControlMode.PercentOutput, v);
    }

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
          () -> {
            BeltController.set(ControlMode.PercentOutput, d.getAsDouble());
          },
          () -> {
            BeltController.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Grasper");
    }
    
}