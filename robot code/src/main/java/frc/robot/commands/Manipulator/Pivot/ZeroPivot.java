package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MiniSystems.Pivot;

public class ZeroPivot extends CommandBase{
    private Pivot mPivot;
    public ZeroPivot(Pivot pivot){
        mPivot = pivot;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute(){
        mPivot.zeroPivot(true);
    }
    @Override
    public void end(boolean interrupted){
    }
}
