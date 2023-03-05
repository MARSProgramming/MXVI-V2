package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class SetFlashing extends CommandBase{
    private LED Lights;
    public SetFlashing(LED m){
        Lights = m;
    }

    @Override
    public void execute(){
        Lights.startFlashing();
    
    }

    @Override
    public void end(boolean interrupted){
       Lights.stopFlashing();
    }
}
