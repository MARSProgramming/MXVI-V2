package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class PivotToShootHigh extends CommandBase{
    private Manipulator manipulator;
    public PivotToShootHigh(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getElevator().getPosition() > 2){
            manipulator.getPivot().goToShootHigh();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }
}
