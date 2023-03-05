package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BtP1P2_Dock extends SequentialCommandGroup{
    public BtP1P2_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1", new PathConstraints(4, 3));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkertoP2 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P2", new PathConstraints(4, 3));
        PathPlannerTrajectory P2toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P2-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkerToDock = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-C", new PathConstraints(4, 3));
        addCommands(
            new ResetDrivePose(drivetrain, 1.83, 4.34, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP1, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P1toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP2, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P2toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkerToDock, false, false, 10)
                // Any changes to prepare for Teleop here
            )

        );
    }
}
