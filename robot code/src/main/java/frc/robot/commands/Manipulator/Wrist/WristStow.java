package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class WristStow extends CommandBase{
    private Manipulator manipulator;
    public WristStow(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getPivot().getEncoderPos() > -0.4){
            manipulator.getWrist().goToStow();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getWrist().distanceToSetpoint(Constants.Wrist.stowPos)) < 0.1;
    }
}
