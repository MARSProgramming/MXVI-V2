package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class MidLeaveCommunityDock extends SequentialCommandGroup{
    public MidLeaveCommunityDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFile("BLUE_MiddleLeaveCommunity", new PathConstraints(1.5, 0.75));
        PathPlannerTrajectory Dock = AutoChooser.openTrajectoryFile("BLUE_MiddleLeaveCommunity", new PathConstraints(1, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            new DriveAtPath(drivetrain, LeaveCommunity, false, false, 10),
            new DriveAtPath(drivetrain, Dock, false, false, 10),
            new AutoBalance(drivetrain)
        );
    }
}
