package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class PivotToScoreHigh extends CommandBase{
    private Manipulator manipulator;
    public PivotToScoreHigh(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getElevator().getPosition() > 5){
            manipulator.getPivot().goToScoreHigh();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }
}
