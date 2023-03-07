//not done

package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
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
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.83, 0.96, 180),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
                mManipulator.getGrasper().runSpitMode().withTimeout(2),
                mManipulator.goToZero()
         );
    }
}
