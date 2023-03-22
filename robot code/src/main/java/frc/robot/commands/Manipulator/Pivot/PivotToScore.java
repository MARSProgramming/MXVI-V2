package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class PivotToScore extends CommandBase{
    private Manipulator manipulator;
    public PivotToScore(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getElevator().getPosition() > 2){
            manipulator.getPivot().goToScoreMid();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }
}
