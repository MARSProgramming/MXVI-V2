package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Manipulator.Elevator.ElevatorCloseIntake;
import frc.robot.commands.Manipulator.Elevator.ElevatorIntake;
import frc.robot.commands.Manipulator.Elevator.ElevatorIntakeHigh;
import frc.robot.commands.Manipulator.Elevator.ElevatorLoad;
import frc.robot.commands.Manipulator.Elevator.ElevatorLoadDouble;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreHigh;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreLow;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreMid;
import frc.robot.commands.Manipulator.Elevator.ElevatorShootHigh;
import frc.robot.commands.Manipulator.Elevator.ElevatorStow;
import frc.robot.commands.Manipulator.Grasper.ScoreAtPivotElevatorSetpoint;
import frc.robot.commands.Manipulator.Grasper.ScoreAtPivotSetpoint;
import frc.robot.commands.Manipulator.Pivot.PivotToCubeCloseIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToCubeIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToHighIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToIntake;
import frc.robot.commands.Manipulator.Pivot.PivotToLoad;
import frc.robot.commands.Manipulator.Pivot.PivotToLoadDouble;
import frc.robot.commands.Manipulator.Pivot.PivotToScore;
import frc.robot.commands.Manipulator.Pivot.PivotToScoreHigh;
import frc.robot.commands.Manipulator.Pivot.PivotToScoreLow;
import frc.robot.commands.Manipulator.Pivot.PivotToShootHigh;
import frc.robot.commands.Manipulator.Pivot.PivotToShootMid;
import frc.robot.commands.Manipulator.Pivot.PivotToStow;
import frc.robot.commands.Manipulator.Pivot.PivotToZero;
import frc.robot.commands.Manipulator.Wrist.WristCarry;
import frc.robot.commands.Manipulator.Wrist.WristCubeCloseIntake;
import frc.robot.commands.Manipulator.Wrist.WristCubeIntake;
import frc.robot.commands.Manipulator.Wrist.WristHighIntake;
import frc.robot.commands.Manipulator.Wrist.WristIntake;
import frc.robot.commands.Manipulator.Wrist.WristScoreHigh;
import frc.robot.commands.Manipulator.Wrist.WristScoreLow;
import frc.robot.commands.Manipulator.Wrist.WristScoreMid;
import frc.robot.commands.Manipulator.Wrist.WristStow;
import frc.robot.commands.Manipulator.Wrist.WristToLoad;
import frc.robot.commands.Manipulator.Wrist.WristToLoadDouble;
import frc.robot.commands.Manipulator.Wrist.WristToShoot;
import frc.robot.commands.Manipulator.Wrist.WristToShootHigh;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;

public class Manipulator extends SubsystemBase{
    private Elevator mElevator = new Elevator();
    private Grasper mGrasper = new Grasper();
    private Pivot mPivot = new Pivot();
    private Wrist mWrist = new Wrist();
    private boolean autoScore = false;
    private int alignType = 1;
    
    public boolean getAutoScore(){
        return autoScore;
    }

    public CommandBase swapAutoScoreCommand(){
        return runOnce(
            () -> {
                if(autoScore){autoScore = false;}
                else{autoScore = true;}
            }
        );
    }

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
        new PivotToShootMid(this).alongWith(new WristToShoot(this))
        );
        shootCommand.addRequirements(this);
        return shootCommand;
    }

    public CommandBase goToScoreHigh() {
        CommandBase scoreHighCommand = Commands.sequence(
            new ScoreAtPivotElevatorSetpoint(this, Constants.Pivot.scoreHighPos, Constants.Elevator.scoreHighPos).deadlineWith(
                new ElevatorScoreHigh(this).alongWith(new WaitCommand(0.4).andThen(new PivotToScoreHigh(this).alongWith(new WristScoreHigh(this))))
            ),
            goToZero()
        );
        scoreHighCommand.addRequirements(this);

        return scoreHighCommand;
    }
    
    public CommandBase goToCubeShootHigh() {
        CommandBase scoreHighCommand = Commands.sequence(
            new ParallelCommandGroup(
                new ElevatorShootHigh(this), 
                new PivotToShootHigh(this),
                new WristToShootHigh(this)
            )
        );
        scoreHighCommand.addRequirements(this);

        return scoreHighCommand;
    }

    public CommandBase goToHighIntake() {
        CommandBase intakeHighCommand = 
        new ParallelCommandGroup(
            new ElevatorIntakeHigh(this), 
            new PivotToHighIntake(this), 
            new WristHighIntake(this)
        );
        intakeHighCommand.addRequirements(this);
        return intakeHighCommand;
    }

    public CommandBase goToScoreMid() {
        CommandBase scoreMidCommand = Commands.sequence(
            new ParallelCommandGroup(
                new ElevatorScoreMid(this),
                new ScoreAtPivotElevatorSetpoint(this, Constants.Pivot.scoreMidPos, Constants.Elevator.scoreMidPos).deadlineWith(
                    new PivotToScore(this), new WristScoreMid(this)
                )
            ), 
            goToZero()
        );
        scoreMidCommand.addRequirements(this);
        return scoreMidCommand;
    }

    public int getAlignType(){
        return alignType;
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

    public CommandBase goToCloseCubeIntake() {
        CommandBase intakeCubeCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorCloseIntake(this), 
            new PivotToCubeCloseIntake(this), 
            new WristCubeCloseIntake(this)
        )
        );
        intakeCubeCommand.addRequirements(this);
        return intakeCubeCommand;
    }

    public CommandBase goToLoadCommand() {
        CommandBase loadCommand = Commands.sequence(
        new WristCarry(this), 
        new ParallelCommandGroup(
            new ElevatorLoad(this), 
            new PivotToLoad(this), 
            new WristToLoad(this)
        )
        );
        loadCommand.addRequirements(this);
        return loadCommand;
    }
    public CommandBase goToStow() {
        CommandBase loadCommand = Commands.sequence(
            new ElevatorStow(this).alongWith(
            new WristCarry(this),
            new PivotToZero(this)),
            new WristStow(this),
            new PivotToStow(this)
        );
        loadCommand.addRequirements(this);
        return loadCommand;
    }
    public CommandBase goToLoadDouble() {
        CommandBase loadCommand = Commands.parallel(
            new ElevatorLoadDouble(this),
            new WristToLoadDouble(this),
            new PivotToLoadDouble(this)
        );
        loadCommand.addRequirements(this);
        return loadCommand;
    }

    public CommandBase goToScoreLow(){
        return new WristCarry(this).andThen(Commands.parallel(
            new WristScoreLow(this), new ElevatorScoreLow(this), new PivotToScoreLow(this)
        ));
    }
    public void setAlignType(int d){
        alignType = d;
        System.out.println(d);
    }

    public CommandBase setAlignTypeCommand(int d){
        return runOnce(
            () -> {
                setAlignType(d);
            }
        );
    }
    public CommandBase goToZero(){
        return runEnd(
            () -> {
                if(mPivot.getEncoderPos() > 0.5){
                    mWrist.setPosition(1);
                }
                else{
                    mWrist.goToCarry();
                }
                mElevator.goToBottom();
                if(mPivot.getEncoderPos() > -0.4 || mElevator.getPosition() > 4 || Math.abs(mWrist.distanceToSetpoint(Constants.Wrist.carryPos)) < 0.1){
                    mPivot.setpos(-0.1);
                }
            },
            () -> {
                mWrist.setPercentOutput(0);
                mElevator.setPercentOutput(0);
                mPivot.Run(0);
            }
        );
        }

    @Override
    public void periodic(){
        double forward = Constants.Wrist.forwardLimit + mPivot.getEncoderPos() * 2;
        double reverse = Constants.Wrist.reverseLimit + mPivot.getEncoderPos() * 2;
        mWrist.configureLimits(forward, reverse);
    }
}
