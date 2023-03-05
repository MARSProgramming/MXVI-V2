package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;

public class BtScoreGiven extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public BtScoreGiven(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);

        addCommands(
            new ResetDrivePose(drivetrain, 1.83, 4.34, 0),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(2),
            mManipulator.goToZero()
         );
    }
}
