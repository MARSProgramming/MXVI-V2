package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class WristToShootHigh extends CommandBase{
    private Manipulator manipulator;
    public WristToShootHigh(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getElevator().getPosition() > 2){
            manipulator.getWrist().goToShootHigh();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getWrist().distanceToSetpoint(Constants.Wrist.shootHighPos)) < 0.1;
    }
}
