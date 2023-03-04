package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;

public class BbScoreGiven extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    private Manipulator mManipulator;
    public BbScoreGiven(DrivetrainSubsystem drivetrain, Manipulator manipulator){
        mDrivetrain = drivetrain;
        mManipulator = manipulator;
        addRequirements(drivetrain, manipulator);

        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
                mManipulator.goToScoreMid().withTimeout(6),
                mManipulator.getGrasper().runSpitMode().withTimeout(2),
                mManipulator.goToZero()
         );
    }
}
