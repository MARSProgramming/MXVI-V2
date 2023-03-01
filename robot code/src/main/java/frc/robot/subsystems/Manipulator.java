package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Manipulator.Elevator.ElevatorIntake;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreHigh;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreMid;
import frc.robot.commands.Manipulator.Pivot.PivotToHighIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToLoad;
import frc.robot.commands.Manipulator.Pivot.PivotToScore;
import frc.robot.commands.Manipulator.Wrist.WristCarry;
import frc.robot.commands.Manipulator.Wrist.WristCubeIntake;
import frc.robot.commands.Manipulator.Wrist.WristHighIntake;
import frc.robot.commands.Manipulator.Wrist.WristIntake;
import frc.robot.commands.Manipulator.Wrist.WristScoreHigh;
import frc.robot.commands.Manipulator.Wrist.WristScoreMid;
import frc.robot.commands.Manipulator.Wrist.WristToLoad;
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

    private CommandBase intakeCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorIntake(this), 
            new PivotToIntake(this), 
            new WristIntake(this)
        )
    );
    private CommandBase intakeHighCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorIntake(this), 
            new PivotToHighIntake(this), 
            new WristHighIntake(this)
        )
    );
    private CommandBase intakeCubeCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorIntake(this), 
            new PivotToHighIntake(this), 
            new WristCubeIntake(this)
        )
    );

    private CommandBase scoreHighCommand = Commands.sequence(
        new WristCarry(this), 
        new ElevatorScoreHigh(this),
        new ParallelCommandGroup( 
            new PivotToScore(this), 
            new WristScoreHigh(this)
        )
    );

    private CommandBase scoreMidCommand = Commands.sequence(
        new ParallelCommandGroup(new WristCarry(this), 
        new ElevatorScoreMid(this)),
        new ParallelCommandGroup( 
            new PivotToScore(this), 
            new WristScoreMid(this)
        )
    );
    
    private CommandBase loadCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorIntake(this), 
            new PivotToLoad(this), 
            new WristToLoad(this)
        )
    );
    
    public Manipulator(){
        intakeCommand.addRequirements(this);
        scoreHighCommand.addRequirements(this);
        scoreMidCommand.addRequirements(this);
        intakeHighCommand.addRequirements(this);
        intakeCubeCommand.addRequirements(this);
        loadCommand.addRequirements(this);
    }

    public CommandBase goToIntake() {
        return intakeCommand;
    }

    public CommandBase goToScoreHigh() {
        return scoreHighCommand;
    }

    public CommandBase goToHighIntake() {
        return intakeHighCommand;
    }

    public CommandBase goToScoreMid() {
        return scoreMidCommand;
    }

    public CommandBase goToCubeIntake() {
        return intakeCubeCommand;
    }

    public CommandBase goToLoadCommand() {
        return loadCommand;
    }

    public CommandBase cancelSetpointCommands(){
        return runOnce(
            () -> {
                intakeCommand.cancel();
                scoreHighCommand.cancel();
                scoreMidCommand.cancel();
                intakeHighCommand.cancel();
                intakeCubeCommand.cancel();
                loadCommand.cancel();
            }
        );
    }

    public CommandBase goToZero(){
        return runEnd(
            () -> {
                mElevator.goToBottom();
                mPivot.setpos(0);
            },
            () -> {
                mElevator.setPercentOutput(0);
                mPivot.Run(0);
            }
        );
    }
}
