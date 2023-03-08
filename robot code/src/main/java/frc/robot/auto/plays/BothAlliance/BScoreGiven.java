//not done

package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BScoreGiven extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    private Manipulator mManipulator;

    PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForAlliance("BLUE_BottomLeaveCommunity", new PathConstraints(1.0, 0.5));
    public BScoreGiven(DrivetrainSubsystem drivetrain, Manipulator manipulator){
        mDrivetrain = drivetrain;
        mManipulator = manipulator;
        addRequirements(drivetrain, manipulator);

        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
                mManipulator.getGrasper().runSpitMode().withTimeout(2),
                mManipulator.goToZero()
         );
    }
}
