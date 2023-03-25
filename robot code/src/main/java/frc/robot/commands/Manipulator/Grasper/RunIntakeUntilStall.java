package frc.robot.commands.Manipulator.Grasper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class RunIntakeUntilStall extends CommandBase{
    private Manipulator mManipulator;

    public RunIntakeUntilStall(Manipulator m){
        mManipulator = m;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        mManipulator.getGrasper().RunGrasperStallcheck();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return mManipulator.getGrasper().getStatorCurrent() > 20;
    }
}
