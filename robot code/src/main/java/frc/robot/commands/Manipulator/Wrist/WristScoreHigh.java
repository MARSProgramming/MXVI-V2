package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class WristScoreHigh extends CommandBase{
    private Manipulator manipulator;
    public WristScoreHigh(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getWrist().goToScoreHigh();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getWrist().distanceToSetpoint(Constants.Wrist.scoreHighPos)) < 0.1;
    }
}
