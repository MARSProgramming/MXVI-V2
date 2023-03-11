package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class PivotToStow extends CommandBase{
    private Manipulator manipulator;
    public PivotToStow(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getPivot().goToStow();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getPivot().distanceToSetpoint(Constants.Pivot.stowPos)) < 0.03;
    }
}
