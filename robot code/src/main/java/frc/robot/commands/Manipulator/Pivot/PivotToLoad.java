package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class PivotToLoad extends CommandBase{
    private Manipulator manipulator;
    public PivotToLoad(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getPivot().goToLoad();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }
}
