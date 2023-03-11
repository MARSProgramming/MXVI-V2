package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.LED.SetFlashing;
import frc.robot.commands.Manipulator.Elevator.ElevatorIntake;
import frc.robot.commands.Manipulator.Elevator.ElevatorIntakeHigh;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreHigh;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreMid;
import frc.robot.commands.Manipulator.Elevator.ElevatorStow;
import frc.robot.commands.Manipulator.Grasper.ScoreAtPivotSetpoint;
import frc.robot.commands.Manipulator.Pivot.PivotToCubeIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToHighIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToLoad;
import frc.robot.commands.Manipulator.Pivot.PivotToScore;
import frc.robot.commands.Manipulator.Pivot.PivotToShootHigh;
import frc.robot.commands.Manipulator.Pivot.PivotToStow;
import frc.robot.commands.Manipulator.Pivot.PivotToZero;
import frc.robot.commands.Manipulator.Wrist.WristCarry;
import frc.robot.commands.Manipulator.Wrist.WristCubeIntake;
import frc.robot.commands.Manipulator.Wrist.WristHighIntake;
import frc.robot.commands.Manipulator.Wrist.WristIntake;
import frc.robot.commands.Manipulator.Wrist.WristScoreHigh;
import frc.robot.commands.Manipulator.Wrist.WristScoreMid;
import frc.robot.commands.Manipulator.Wrist.WristStow;
import frc.robot.commands.Manipulator.Wrist.WristToLoad;
import frc.robot.commands.Manipulator.Wrist.WristToShoot;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;

public class Manipulator extends SubsystemBase{
    private Elevator mElevator = new Elevator();
    private Grasper mGrasper = new Grasper();
    private Pivot mPivot = new Pivot();
    private Wrist mWrist = new Wrist();
    private LED mLED;
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

    public Manipulator(LED subsystemled){ 
        mLED = subsystemled;
    }

    public CommandBase goToIntake() {
        CommandBase intakeCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorIntake(this), 
            new PivotToIntake(this), 
            new WristIntake(this)
        )
        );
        intakeCommand.addRequirements(this);
        return intakeCommand;
    }
    public CommandBase goToShoot() {
        CommandBase shootCommand = Commands.sequence(
        new WristCarry(this),
        mPivot.goToZeroCommand(),
        new WristToShoot(this)
        );
        shootCommand.addRequirements(this);
        return shootCommand;
    }

    public CommandBase goToScoreHigh() {
        CommandBase scoreHighCommand = Commands.sequence(
            new ParallelCommandGroup(new WristScoreHigh(this), new ParallelCommandGroup(new ElevatorScoreHigh(this), new PivotToScore(this)))
        );
        scoreHighCommand.addRequirements(this);

        return scoreHighCommand;
    }
    
    public CommandBase goToCubeShootHigh() {
        CommandBase scoreHighCommand = Commands.sequence(
            new ParallelCommandGroup(new ElevatorScoreMid(this), 
            new PivotToShootHigh(this).alongWith(new WristToShoot(this)))
        );
        scoreHighCommand.addRequirements(this);

        return scoreHighCommand;
    }

    public CommandBase goToHighIntake() {
        CommandBase intakeHighCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorIntakeHigh(this), 
            new PivotToHighIntake(this), 
            new WristHighIntake(this)
        )
        );    
        intakeHighCommand.addRequirements(this);
        return intakeHighCommand;
    }

    public CommandBase goToScoreMid() {
        CommandBase scoreMidCommand = Commands.sequence(
        new ParallelCommandGroup(new ElevatorScoreMid(this),
        new PivotToScore(this).alongWith(new WristScoreMid(this), new ScoreAtPivotSetpoint(this, Constants.Pivot.scoreHighPos))
        ));
        scoreMidCommand.addRequirements(this);
        return scoreMidCommand;
    }

    public CommandBase goToCubeIntake() {
        CommandBase intakeCubeCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            mElevator.testBottomSetpoint(), 
            new PivotToCubeIntake(this), 
            new WristCubeIntake(this)
        )
        );
        intakeCubeCommand.addRequirements(this);
        return intakeCubeCommand;
    }

    public CommandBase goToLoadCommand() {
        CommandBase loadCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            mElevator.testBottomSetpoint(), 
            new PivotToLoad(this), 
            new WristToLoad(this)  
            ),
        new SetFlashing(mLED)

        );
        loadCommand.addRequirements(this);
        return loadCommand;
    }
    public CommandBase goToStow() {
        CommandBase loadCommand = Commands.sequence(
            new ElevatorStow(this), 
            new WristCarry(this),
            new PivotToZero(this),
            new WristStow(this),
            new PivotToStow(this)
        );
        loadCommand.addRequirements(this);
        return loadCommand;
    }

    public CommandBase goToZero(){
        return runEnd(
            () -> {
                mWrist.goToCarry();
                mElevator.goToBottom();
                mPivot.setpos(0);
            },
            () -> {
                mWrist.setPercentOutput(0);
                mElevator.setPercentOutput(0);
                mPivot.Run(0);
            }
        );
    }
}
