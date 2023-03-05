package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class PivotToZero extends CommandBase{
    private Manipulator manipulator;
    public PivotToZero(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getPivot().setpos(0);
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getPivot().distanceToSetpoint(0)) < 0.1;
    }
}
