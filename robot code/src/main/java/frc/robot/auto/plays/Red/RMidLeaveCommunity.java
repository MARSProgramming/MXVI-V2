package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RMidLeaveCommunity extends SequentialCommandGroup{
    public RMidLeaveCommunity(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openRedTrajectoryFile("BLUE_MiddleLeaveCommunity", new PathConstraints(4, 3));
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 2.69, 0),
            new ParallelCommandGroup(
                // Any changes to make sure arm is secure here
                new DriveAtPath(drivetrain, LeaveCommunity, false, false, 10)
            )

        );
    }
}
