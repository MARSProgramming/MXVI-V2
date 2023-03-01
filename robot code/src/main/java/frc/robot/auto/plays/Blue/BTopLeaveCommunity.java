package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BTopLeaveCommunity extends SequentialCommandGroup{
    public BTopLeaveCommunity(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFile("BLUE_TopLeaveCommunity");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, 0, 10)
            )

        );
    }
}
