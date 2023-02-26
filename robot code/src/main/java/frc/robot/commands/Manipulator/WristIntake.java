package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class WristIntake extends CommandBase{
    private Manipulator manipulator;
    public WristIntake(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        if(manipulator.getPivot().getEncoderPos() > 0){
            manipulator.getWrist().goToIntake();
        }
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getWrist().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getWrist().distanceToSetpoint(Constants.Wrist.intakeBackPos)) < 0.1;
    }
}
