package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BtScoreGivenLeaveDock extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public BtScoreGivenLeaveDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);
        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-C", new PathConstraints(1, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.83, 4.34, 0),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(2),
            new DriveAtPath(drivetrain, LeaveCommunity, true, false, 100).deadlineWith(mManipulator.goToZero())
         );
    }
}
