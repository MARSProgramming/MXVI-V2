package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class RbScoreGivenLeave extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public RbScoreGivenLeave(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);
        // Add stuff here to leave community
        PathPlannerTrajectory LeaveCommunity = AutoChooser.openRedTrajectoryFile("BLUE_BottomLeaveCommunity", new PathConstraints(0.5, 0.5));
        addCommands(
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialPose().getX(), LeaveCommunity.getInitialPose().getY(), LeaveCommunity.getInitialPose().getRotation().getDegrees()),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(2),
            mManipulator.goToZero(),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, false, false, 10)
            )
         );
    }
}
