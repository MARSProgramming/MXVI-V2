package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RBottomLeaveCommunity extends SequentialCommandGroup{
    public RBottomLeaveCommunity(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openRedTrajectoryFile("BLUE_BottomLeaveCommunity", new PathConstraints(0.5, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialPose().getX(), LeaveCommunity.getInitialPose().getY(), LeaveCommunity.getInitialPose().getRotation().getDegrees()),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, false, false, 10)
            )

        );
    }
}
