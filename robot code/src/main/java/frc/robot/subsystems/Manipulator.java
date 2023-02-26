package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Manipulator.ElevatorIntake;
import frc.robot.commands.Manipulator.PivotToIntake;
import frc.robot.commands.Manipulator.WristCarry;
import frc.robot.commands.Manipulator.WristIntake;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;

public class Manipulator extends SubsystemBase{
    private Elevator mElevator = new Elevator();
    private Grasper mGrasper = new Grasper();
    private Pivot mPivot = new Pivot();
    private Wrist mWrist = new Wrist();
    
    public Elevator getElevator(){
        return mElevator;
    }
    public Grasper getGrasper(){
        return mGrasper;
    }
    public Pivot getPivot(){
        return mPivot;
    }
    public Wrist getWrist(){
        return mWrist;
    }

    public Manipulator(){

    }
    
    public CommandBase goToScore() {
        return run(
          () -> {
            if(Math.abs(mPivot.distanceToSetpoint(Constants.Pivot.scorePos)) < 0.1){
                mWrist.goToScore();
            }
            else{
                if(Math.abs(mWrist.distanceToSetpoint(Constants.Wrist.carryPos)) < 0.1){
                    mPivot.goToScore();
                }
                else{
                    mWrist.goToCarry();
                }
            }

          }
          ).withName("To Scoring Position");
    }

    private CommandBase intakeCommand = Commands.sequence(new ParallelCommandGroup(new WristCarry(this), new ElevatorIntake(this)), new ParallelCommandGroup(new PivotToIntake(this), new WristIntake(this)));
    public CommandBase goToIntake() {
        return intakeCommand;
    }

    public CommandBase cancelCommands(){
        return runOnce(
            () -> {
                intakeCommand.cancel();
            }
        );
    }
}
