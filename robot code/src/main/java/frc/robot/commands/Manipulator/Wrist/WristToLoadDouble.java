package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class WristToLoadDouble extends CommandBase{
    private Manipulator manipulator;
    public WristToLoadDouble(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getElevator().getPosition() > 2){
            manipulator.getWrist().goToLoadDouble();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }
}
