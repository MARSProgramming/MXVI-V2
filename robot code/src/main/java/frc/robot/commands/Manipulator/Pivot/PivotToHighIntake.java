package frc.robot.commands.Manipulator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class PivotToHighIntake extends CommandBase{
    private Manipulator manipulator;
    public PivotToHighIntake(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
            manipulator.getPivot().goToIntakeHigh();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getPivot().Run(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getPivot().distanceToSetpoint(Constants.Pivot.intakeHighPos)) < 0.03;
    }
}
