package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;

public class Manipulator extends SubsystemBase{
    private Elevator mElevator = new Elevator();
    private Grasper mGrasper = new Grasper();
    private Pivot mPivot = new Pivot();
    private Wrist mWrist = new Wrist();
    
    public Elevator getElevator(){
        return mElevator;
    }
    public Grasper getGrasper(){
        return mGrasper;
    }
    public Pivot getPivot(){
        return mPivot;
    }
    public Wrist getWrist(){
        return mWrist;
    }

    public Manipulator(){
    }

    
}
