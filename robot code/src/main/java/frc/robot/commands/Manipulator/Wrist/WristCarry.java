package frc.robot.commands.Manipulator.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class WristCarry extends CommandBase{
    private Manipulator manipulator;
    public WristCarry(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getWrist().goToCarry();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getWrist().distanceToSetpoint(Constants.Wrist.carryPos)) < 0.1 || manipulator.getPivot().getEncoderPos() > 0;
    }
}
