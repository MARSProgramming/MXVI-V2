package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RbP3P4_NoDock extends SequentialCommandGroup{
    public RbP3P4_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);
        
        PathPlannerTrajectory MarkertoP3 = AutoChooser.openRedTrajectoryFile("BLUE_BottomMarker_M-P3", new PathConstraints(4, 3));
        PathPlannerTrajectory P3toMarker = AutoChooser.openRedTrajectoryFile("BLUE_BottomMarker_P3-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkertoP4 = AutoChooser.openRedTrajectoryFile("BLUE_BottomMarker_M-P4", new PathConstraints(4, 3));
        PathPlannerTrajectory P4toMarker = AutoChooser.openRedTrajectoryFile("BLUE_BottomMarker_P4-M", new PathConstraints(4, 3));
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP3, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P3toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP4, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P4toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
                // Any changes to prepare for Teleop here
            )

        );
    }
}
