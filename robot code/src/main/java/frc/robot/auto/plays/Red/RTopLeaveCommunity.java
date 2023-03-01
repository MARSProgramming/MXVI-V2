package frc.robot.auto.plays.Red;

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

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFile("RED_TopLeaveCommunity");
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 4.37, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, 0, 10)
            )

        );
    }
}
