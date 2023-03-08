package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class MidSetPose extends SequentialCommandGroup{

    PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForAlliance("BLUE_MiddleLeaveCommunity", new PathConstraints(1, 0.5));
    public MidSetPose(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        addCommands(
            new ZeroGyroscope(drivetrain, 180),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose())
        );
    }
}
