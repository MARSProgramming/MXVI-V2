package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RTopLeaveCommunity extends SequentialCommandGroup{
    public RTopLeaveCommunity(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openRedTrajectoryFile("BLUE_TopLeaveCommunity", new PathConstraints(4, 3));
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 4.31, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, false, false, 10)
            )

        );
    }
}
