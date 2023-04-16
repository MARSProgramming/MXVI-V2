package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BLUE_MID_SCORELEAVEDOCK extends SequentialCommandGroup{
    public BLUE_MID_SCORELEAVEDOCK(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForBlue("BLUE_MID_LEAVE", new PathConstraints(1, 0.5));
        PathPlannerTrajectory Dock = AutoChooser.openTrajectoryFileForBlue("BLUE_MID_LEAVE-DOCK", new PathConstraints(1, 1));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(3),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            new DriveAtPath(drivetrain, LeaveCommunity, false, false, 5).deadlineWith(mManipulator.goToZero()),
            new DriveAtPath(drivetrain, Dock, false, false, 3),
            new AutoBalance(drivetrain)
        );
    }
}
