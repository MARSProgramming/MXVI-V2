package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class PivotToLoadDouble extends CommandBase{
    private Manipulator manipulator;
    public PivotToLoadDouble(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getPivot().goToLoadDouble();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }
}
