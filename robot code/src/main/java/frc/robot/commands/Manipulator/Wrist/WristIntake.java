package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class WristIntake extends CommandBase{
    private Manipulator manipulator;
    public WristIntake(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getPivot().getEncoderPos() > 0.5){
            manipulator.getWrist().goToIntake();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }
}
