package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RtP1P2_Dock extends SequentialCommandGroup{
    public RtP1P2_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("RED_TopMarker_M-P1", new PathConstraints(4, 3));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("RED_TopMarker_P1-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkertoP2 = AutoChooser.openTrajectoryFile("RED_TopMarker_M-P2", new PathConstraints(4, 3));
        PathPlannerTrajectory P2toMarker = AutoChooser.openTrajectoryFile("RED_TopMarker_P2-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkerToDock = AutoChooser.openTrajectoryFile("RED_TopMarker_M-C", new PathConstraints(4, 3));
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 4.37, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP1, 0, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P1toMarker, 0, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP2, 0, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P2toMarker, 0, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkerToDock, 0, 10)
                // Any changes to prepare for Teleop here
            )

        );
    }
}
