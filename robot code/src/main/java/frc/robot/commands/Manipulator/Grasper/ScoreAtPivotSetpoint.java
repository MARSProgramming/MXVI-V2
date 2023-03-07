package frc.robot.commands.Manipulator.Grasper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class ScoreAtPivotSetpoint extends CommandBase{
    private Manipulator mManipulator;
    private double setpoint;
    private Timer timer = new Timer();

    public ScoreAtPivotSetpoint(Manipulator m, double s){
        mManipulator = m;
        setpoint = s;
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        if(Math.abs(mManipulator.getPivot().distanceToSetpoint(setpoint)) < 0.07 && mManipulator.getAutoScore()){
            timer.start();
            mManipulator.getGrasper().RunGrasperEject();
        }
    }

    @Override
    public void end(boolean interrupted){
        timer.stop();
        mManipulator.getGrasper().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return timer.get() > 0.4;
    }
}
