package frc.robot.commands.Manipulator.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class ElevatorStow extends CommandBase{
    private Manipulator manipulator;
    public ElevatorStow(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getElevator().goToStow();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getElevator().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getElevator().distanceToSetpoint(Constants.Elevator.stowPos)) < 0.4;
    }
}
