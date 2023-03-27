package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class AlignToScoreManipulator extends CommandBase{
    private Manipulator mManipulator;
    private int alignType;
    public AlignToScoreManipulator(Manipulator m){
        mManipulator = m;
        addRequirements(m);
    }

    @Override
    public void initialize(){
        alignType = mManipulator.getAlignType();
    }

    @Override
    public void execute(){
        if(alignType == 0){
            mManipulator.getElevator().setPosition(5);
            mManipulator.getPivot().goToShootMid();
            mManipulator.getWrist().goToScoreHigh();
        }
        else if(alignType == 1){
            mManipulator.getElevator().goToScoreMid();
            mManipulator.getPivot().goToScoreMid();
            mManipulator.getWrist().goToScoreMid();
        }
        else if(alignType == 2){
            mManipulator.getElevator().goToScoreHigh();
            mManipulator.getPivot().goToScoreHigh();
            mManipulator.getWrist().goToScoreHigh();
        }
    }

    @Override
    public void end(boolean interrupted){
        mManipulator.getElevator().setPercentOutput(0);
        mManipulator.getPivot().Run(0);
        mManipulator.getWrist().setPercentOutput(0);
    }
}
