package frc.robot.commands.Manipulator.Grasper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class ScoreAtPivotElevatorSetpoint extends CommandBase{
    private Manipulator mManipulator;
    private double pivotSetpoint;
    private double elevatorSetpoint;
    private Timer timer = new Timer();

    public ScoreAtPivotElevatorSetpoint(Manipulator m, double p, double e){
        mManipulator = m;
        pivotSetpoint = p;
        elevatorSetpoint = e;
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        if(Math.abs(mManipulator.getElevator().distanceToSetpoint(elevatorSetpoint)) < 0.4 && Math.abs(mManipulator.getPivot().distanceToSetpoint(pivotSetpoint)) < 0.15 && mManipulator.getAutoScore()){
            timer.start();
            mManipulator.getGrasper().setPercentOutput(-1);
        }
    }

    @Override
    public void end(boolean interrupted){
        timer.stop();
        mManipulator.getGrasper().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return timer.get() > 0.3;
    }
}
