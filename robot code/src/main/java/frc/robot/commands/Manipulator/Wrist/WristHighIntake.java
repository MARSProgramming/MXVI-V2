package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class WristHighIntake extends CommandBase{
    private Manipulator manipulator;
    public WristHighIntake(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getPivot().getEncoderPos() > 1){
            manipulator.getWrist().goToHighIntake();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }
}
