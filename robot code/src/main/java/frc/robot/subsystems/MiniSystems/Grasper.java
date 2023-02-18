// This subsystem is the part of the robot that actually picks up cones and squares, with no sensor it will have to read va;ues to determine whether cargo has been picked up


package frc.robot.subsystems.MiniSystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Grasper extends SubsystemBase{

    private TalonFX BeltController = new TalonFX(9);
    private double[] PositionTable = new double[]{10000,10000,10000,10000,10000,10000,10000,10000,10000,10000};
    private int count = 0;
    public Grasper() {
        BeltController.setNeutralMode(NeutralMode.Brake);
    }

    public void RunGrasperIntake(){

        double Average = 0;
        for(int i = 0; i <= PositionTable.length; i++){
            Average += PositionTable[i];
        }

        Average = Average / PositionTable.length;

        if(Math.abs(BeltController.getSelectedSensorPosition() - Average) < 500){
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

}