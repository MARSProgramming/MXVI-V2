package frc.robot.commands.Manipulator.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class ElevatorCloseIntake extends CommandBase{
    private Manipulator manipulator;
    public ElevatorCloseIntake(Manipulator m){
        manipulator = m;
    }

    @Override
    public void execute(){
        manipulator.getElevator().goToCloseCubeIntake();
    }

    @Override
    public void end(boolean interrupted){
        manipulator.getElevator().setPercentOutput(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(manipulator.getElevator().distanceToSetpoint(Constants.Elevator.intakeCloseCubePos)) < 0.1;
    }
}
